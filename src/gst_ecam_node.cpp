#include "ros/ros.h"
#include <std_msgs/Time.h>
#include <math.h>

extern "C"
{
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <glib.h>
}

#include "boost/date_time/posix_time/posix_time.hpp"
#include <fstream>
#include <sstream>

#define DEBUG 0

guint64 start_time_ns = 0;
std::ofstream csv_file;
std::stringstream csv_buffer;

// Declaration
std::string get_timestamp_as_string(ros::Time rosTime, int hoursOffset);
bool _replace(std::string &str, const std::string &from, const std::string &to);
void _replaceAll(std::string &str, const std::string &from, const std::string &to);

typedef struct _GstElements
{
    GstElement *pipeline;
    GstElement *source;
    GstElement *tee;
    GstElement *queue1;
    GstElement *queue2;
    GstElement *nvvidconv1;
    GstElement *nvvidconv2;
    GstElement *enc;
    GstElement *h264parse;
    GstElement *mux;
    GstElement *sink;
    GstElement *appsink;
    GstElement *capsfilter1;
    GstElement *capsfilter2;
    GstElement *capsfilterEncoder;

} GstElements;

std::string rosTimeToString(const ros::Time &time)
{
    std::stringstream ss;
    ss << time.sec << "." << std::setw(9) << std::setfill('0') << time.nsec;
    return ss.str();
}

/**
 * Process a new sample received from an AppSink.
 *
 * @param sink The AppSink that received the sample.
 * @param data Additional data associated with the sample processing.
 * @return The status of the sample processing operation.
 *
 * @note This function pulls a sample from the given AppSink, retrieves its buffer and timestamp,
 * calculates the elapsed time since the program start, and optionally prints debug information
 * including the kernel timestamp and epoch time. It then adds the epoch time to a CSV buffer.
 */
static GstFlowReturn new_sample(GstAppSink *sink, gpointer data)
{
    GstSample *sample = gst_app_sink_pull_sample(sink);
    GstBuffer *buffer = gst_sample_get_buffer(sample);

    // Get timestamp of Gst buffer
    GstClockTime buffer_time = GST_BUFFER_PTS(buffer);

    // Get current ros time
    ros::Time current_ros_time = ros::Time::now();

    // Format time as uint64_t
    uint64_t time_as_ros_uint64 = current_ros_time.toNSec();

    // Ensure start_time_ns is initialized
    if (start_time_ns == 0)
    {
        // Get epoch time in nanoseconds
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        start_time_ns = ts.tv_sec * G_GUINT64_CONSTANT(1000000000) + ts.tv_nsec;
    }

    // Calculate elapsed time since program start
    guint64 elapsed_time_ns = 0;
    if (start_time_ns != 0 && buffer_time != GST_CLOCK_TIME_NONE)
    {
        elapsed_time_ns = start_time_ns + buffer_time;
    }

    if (DEBUG)
    {
        // Print Kernel Timestamp and Epoch Time
        g_print("Kernel Timestamp: %" GST_TIME_FORMAT " - Epoch Time: %" G_GUINT64_FORMAT "\n",
                GST_TIME_ARGS(buffer_time), elapsed_time_ns);
    }

    // Add epoch time to buffer
    csv_buffer << elapsed_time_ns << ',' << time_as_ros_uint64 << std::endl;

    gst_sample_unref(sample);

    return GST_FLOW_OK;
}

/**
 * Callback function to handle messages on the GStreamer bus.
 *
 * @param bus The GStreamer bus instance.
 * @param msg The GStreamer message received.
 * @param data Additional data associated with the bus callback.
 * @return gboolean TRUE if the message was handled successfully, FALSE otherwise.
 */
static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data)
{
    GMainLoop *loop = (GMainLoop *)data;

    switch (GST_MESSAGE_TYPE(msg))
    {
    case GST_MESSAGE_EOS:
        g_print("End of stream\n");
        g_main_loop_quit(loop);
        break;
    case GST_MESSAGE_ERROR:
    {
        gchar *debug;
        GError *error;

        gst_message_parse_error(msg, &error, &debug);
        g_free(debug);

        g_printerr("Error: %s\n", error->message);
        g_error_free(error);

        g_main_loop_quit(loop);
        break;
    }
    default:
        break;
    }

    return TRUE;
}

/**
 * Flushes the CSV buffer to the CSV file.
 *
 * @param data Additional data associated with the buffer flushing.
 * @return gboolean TRUE to continue processing, FALSE to stop.
 */
static gboolean flush_csv_buffer(gpointer data)
{
    csv_file << csv_buffer.str();
    csv_file.flush();
    // Clear buffer
    csv_buffer.str(std::string());

    return G_SOURCE_CONTINUE;
}

/**
 * @brief Periodically checks the status of the ROS node and handles shutdown if necessary.
 *
 * This static function is intended to be called repeatedly in a GMainLoop. It checks if the ROS
 * node is still running using `ros::ok()`. If the ROS node is no longer running, the function
 * flushes any remaining data by calling `flush_csv_buffer(nullptr)`, then stops the main loop
 * using `g_main_loop_quit(loop)`. Whether or not the ROS node is running, the function returns
 * `G_SOURCE_CONTINUE` to ensure that the check continues to be scheduled in the main loop.
 *
 * @param loop A pointer to the GMainLoop that manages the event-driven program flow.
 *
 * @return `G_SOURCE_CONTINUE`, indicating that the function should continue to be called
 * periodically by the main loop.
 */
static gboolean check_ros(GMainLoop *loop)
{
    if (!ros::ok())
    {
        flush_csv_buffer(nullptr);
        g_main_loop_quit(loop);
        return G_SOURCE_CONTINUE;
    }
    else
    {
        return G_SOURCE_CONTINUE;
    }
}

/**
 * @brief Converts a ROS time to a timestamp string with an optional hours offset.
 *
 * This function takes a ROS time (`rosTime`) and applies an offset in hours (`hoursOffset`) to it.
 * The adjusted time is then converted to an ISO 8601 extended format string. The resulting string
 * includes the date and time, formatted as "YYYY-MM-DDTHH:MM:SS".
 *
 * @param rosTime The input ROS time to be converted and adjusted.
 * @param hoursOffset The number of hours to add to the input `rosTime`. This can be negative to subtract hours.
 *
 * @return A string representing the adjusted timestamp in ISO 8601 extended format ("YYYY-MM-DDTHH:MM:SS").
 */
std::string get_timestamp_as_string(ros::Time rosTime, int hoursOffset)
{
    rosTime = rosTime + ros::Duration(hoursOffset * 60 * 60);
    return boost::posix_time::to_iso_extended_string(rosTime.toBoost());
}

/**
 * @brief Replaces the first occurrence of a substring within a string with another substring.
 *
 * This function searches for the first occurrence of the substring `from` within the string `str`.
 * If the substring is found, it replaces it with the substring `to`. If the substring `from`
 * is not found within `str`, the function returns `false` and no replacement is made.
 *
 * @param str The string to be searched and modified. The function modifies this string in place.
 * @param from The substring to be replaced. This is the substring that the function searches for within `str`.
 * @param to The substring that will replace the first occurrence of `from` in `str`.
 *
 * @return `true` if the substring `from` was found and replaced; `false` if `from` was not found in `str`.
 */
bool _replace(std::string &str, const std::string &from, const std::string &to)
{
    size_t start_pos = str.find(from);
    if (start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

/**
 * @brief Replaces all occurrences of a substring within a string with another substring.
 *
 * This function searches for all occurrences of the substring `from` within the string `str`.
 * Each occurrence is replaced with the substring `to`. The function continues until all
 * occurrences of `from` are replaced. If `from` is an empty string, the function returns
 * immediately without making any changes.
 *
 * @param str The string to be searched and modified. The function modifies this string in place.
 * @param from The substring to be replaced. This is the substring that the function searches for within `str`.
 * @param to The substring that will replace all occurrences of `from` in `str`.
 *
 * @return None. The function modifies the input string `str` directly.
 */
void _replaceAll(std::string &str, const std::string &from, const std::string &to)
{
    if (from.empty())
        return;
    size_t start_pos = 0;
    while ((start_pos = str.find(from, start_pos)) != std::string::npos)
    {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "gst_ecam_node");
    ros::NodeHandle nh("~");

    std::string sCameraDevice;

    std::string sLocation;
    std::string sRawFileName = "";
    std::string sVideoFileName = "";
    std::string sTextFileName = "";
    std::string sTmpVideoFileEnding = "h264";

    std::string _locationTS;
    std::string _dot = ".";
    std::string sFileNameTimeStamp;
    std::string sStartTime;

    if (!nh.getParam("camera_device", sCameraDevice))
    {
        ROS_WARN("Could not get camera_device! Using fallback!");
        sCameraDevice = "0";
    }
    else
    {
        ROS_INFO("Camera device: %s", sCameraDevice.c_str());
    }

    if (!nh.getParam("record_path", sLocation))
    {
        ROS_WARN("Could not get record path. Using fallback!");
        sLocation = "/storage/bag/";
    }
    else
    {
        ROS_INFO("Storing data to: %s", sLocation.c_str());
    }

    std::cout << sCameraDevice << std::endl;

    // Compose the location string
    sStartTime = get_timestamp_as_string(ros::Time::now(), 2);

    sFileNameTimeStamp = sStartTime;
    sFileNameTimeStamp.append("_");

    sFileNameTimeStamp.append(sCameraDevice);

    _replaceAll(sFileNameTimeStamp, ".", "-");
    _replaceAll(sFileNameTimeStamp, ":", "-");
    _locationTS = sLocation;
    _replace(_locationTS, ".", sFileNameTimeStamp.append(_dot));

    sRawFileName = sLocation + sFileNameTimeStamp.c_str();
    // Append file endings
    sTextFileName = sRawFileName + "txt";
    sVideoFileName = sRawFileName + sTmpVideoFileEnding;

    /* GStreamer */
    GstElements data;
    GstBus *bus;
    GstMessage *msg;
    GstStateChangeReturn ret;
    GstPad *tee_videowriter_pad;
    GstPad *tee_timestamp_pad;
    GstPad *queue_videowriter_pad;
    GstPad *queue_timestamp_pad;

    std::string sVideoDevice = "video";

    // Get device number from command line argument
    std::string devicePath = "/dev/video" + sCameraDevice;

    // Open CSV file for writing
    csv_file.open(sTextFileName);
    if (!csv_file.is_open())
    {
        g_printerr("Failed to open CSV file for writing.\n");
        return -1;
    }

    // Initialize GStreamer
    gst_init(&argc, &argv);

    // Create the elements
    data.source = gst_element_factory_make("nvv4l2camerasrc", "source");
    data.tee = gst_element_factory_make("tee", "splitter");
    data.queue1 = gst_element_factory_make("queue", "queue1");
    data.queue2 = gst_element_factory_make("queue", "queue2");
    data.nvvidconv1 = gst_element_factory_make("nvvidconv", "converter1");
    data.nvvidconv2 = gst_element_factory_make("nvvidconv", "converter2");
    data.enc = gst_element_factory_make("nvv4l2h264enc", "encoder");
    data.h264parse = gst_element_factory_make("h264parse", "parser");
    data.mux = gst_element_factory_make("qtmux", "muxer");
    data.sink = gst_element_factory_make("filesink", "file_sink");

    data.capsfilter1 = gst_element_factory_make("capsfilter", "caps_1");
    data.capsfilter2 = gst_element_factory_make("capsfilter", "caps_2");

    data.capsfilterEncoder = gst_element_factory_make("capsfilter", "caps_Encoder");

    data.appsink = gst_element_factory_make("appsink", "app_sink");

    // Create the empty pipeline
    data.pipeline = gst_pipeline_new("camera-pipeline");

    if (!data.pipeline ||
        !data.source ||
        !data.tee ||
        !data.queue1 ||
        !data.queue2 ||
        !data.nvvidconv1 ||
        !data.nvvidconv2 ||
        !data.enc ||
        !data.h264parse ||
        !data.mux ||
        !data.sink ||
        !data.appsink ||
        !data.capsfilter1 ||
        !data.capsfilter2 ||
        !data.capsfilterEncoder)
    {

        g_printerr("Not all elements could be created.\n");
        return -1;
    }

    // Build the pipeline. Note that we are NOT linking the source at this point. We will do it later.
    gst_bin_add_many(GST_BIN(data.pipeline),
                     data.source, data.tee,
                     data.queue1, data.queue2,
                     data.nvvidconv1, data.nvvidconv2,
                     data.capsfilter1, data.capsfilter2, data.capsfilterEncoder,
                     data.enc, data.h264parse, data.mux,
                     data.sink, data.appsink, NULL);

    if (!gst_element_link_many(data.source, data.tee, NULL) ||
        !gst_element_link_many(data.queue1, data.capsfilter1, data.nvvidconv1, data.capsfilterEncoder, data.enc, data.sink, NULL) ||
        !gst_element_link_many(data.queue2, data.capsfilter2, data.nvvidconv2, data.appsink, NULL))
    {
        g_printerr("Elements could not be linked.\n");
        gst_object_unref(data.pipeline);
        return -1;
    }

    tee_videowriter_pad = gst_element_get_request_pad(data.tee, "src_%u");
    g_print("Obtained request pad %s for video branch.\n", gst_pad_get_name(tee_videowriter_pad));

    tee_timestamp_pad = gst_element_get_request_pad(data.tee, "src_%u");
    g_print("Obtained request pad %s for timestamp branch.\n", gst_pad_get_name(tee_timestamp_pad));

    // videowriter-queue_input_pad
    queue_videowriter_pad = gst_element_get_static_pad(data.queue1, "sink");

    // timestamp-queue_input_pad
    queue_timestamp_pad = gst_element_get_static_pad(data.queue2, "sink");

    if (gst_pad_link(tee_videowriter_pad, queue_videowriter_pad) != GST_PAD_LINK_OK ||
        gst_pad_link(tee_timestamp_pad, queue_timestamp_pad) != GST_PAD_LINK_OK)
    {
        g_printerr("Tee could not be linked.\n");
        gst_object_unref(data.pipeline);
        return -1;
    }

    // Set up appsink callbacks
    GstAppSinkCallbacks callbacks = {NULL, NULL, new_sample, NULL};
    gst_app_sink_set_callbacks(GST_APP_SINK(data.appsink), &callbacks, NULL, NULL);

    GstCaps *caps, *oEncoder_caps;

    g_object_set(G_OBJECT(data.source),
                 "device", devicePath.c_str(),
                 NULL);

    // Set caps for nvvidconv
    caps = gst_caps_new_simple("video/x-raw(memory:NVMM)",
                               "format", G_TYPE_STRING, "UYVY",
                               "width", G_TYPE_INT, 1920,
                               "height", G_TYPE_INT, 1080,
                               "framerate", GST_TYPE_FRACTION, 30, 1,
                               NULL);
    g_object_set(G_OBJECT(data.capsfilter1), "caps", caps, NULL);
    g_object_set(G_OBJECT(data.capsfilter2), "caps", caps, NULL);
    gst_caps_unref(caps);

    // Set caps for enc
    oEncoder_caps = gst_caps_new_simple("video/x-raw(memory:NVMM)",
                                        "format", G_TYPE_STRING, "I420",
                                        "framerate", GST_TYPE_FRACTION, 30, 1,
                                        NULL);
    g_object_set(G_OBJECT(data.capsfilterEncoder), "caps", oEncoder_caps, NULL);
    gst_caps_unref(oEncoder_caps);

    g_object_set(G_OBJECT(data.enc), "control-rate", "1", NULL);

    // std::string filename = "video_" + std::to_string(deviceNumber) + ".h264";
    g_object_set(G_OBJECT(data.sink), "location", sVideoFileName.c_str(), NULL);

    // Start playing
    ret = gst_element_set_state(data.pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        g_printerr("Unable to set the pipeline to the playing state.\n");
        gst_object_unref(data.pipeline);
        return -1;
    }

    GMainLoop *loop = g_main_loop_new(NULL, FALSE);

    // Add bus-Watcher
    bus = gst_element_get_bus(data.pipeline);
    gst_bus_add_watch(bus, bus_call, loop);
    gst_object_unref(bus);

    // Create a timer to flush the CSV buffer every 10 seconds
    g_timeout_add_seconds(10, flush_csv_buffer, NULL);

    g_timeout_add_seconds(1, (GSourceFunc)check_ros, loop);

    // Run main loop
    g_main_loop_run(loop);

    // Free resources
    gst_element_set_state(data.pipeline, GST_STATE_NULL);
    gst_object_unref(data.pipeline);

    // Flush recent timestamps
    if (!csv_buffer.str().empty())
    {
        flush_csv_buffer(nullptr);
    }
    csv_file.close();
}