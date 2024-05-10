#ifndef RADAR_BASE_ECHO_TO_IMAGE_LUT_HPP
#define RADAR_BASE_ECHO_TO_IMAGE_LUT_HPP

#include <opencv2/core.hpp>
#include <vector>

namespace radar_base {
    class EchoToImageLUT {
        /** m_data[angle * sweepSize + cellIndex] contains the list of points in the image
         * that should be filled by the echo with the given angle and cell
         */
        std::vector<cv::Point> m_data;
        std::vector<int> m_data_index;

        int m_sweep_size;
        int m_num_angles;
        float m_beam_width;
        int m_window_size;

        static int normalizeAngle(int angle, int num_angles);

        typedef std::vector<std::vector<cv::Point>> RawTable;
        static RawTable computeRawLUTTable(int sweep_size,
            int window_size,
            int num_angles,
            float beam_width);
        static void addRawLUTEntry(RawTable& table,
            int angle,
            int echo_index,
            int sweep_size,
            int num_angles,
            cv::Point2i const& p);

    public:
        EchoToImageLUT() = delete;
        /**
         * @brief Returns a pair of iterators pointing to the initial and the final pixel
         * in a radar echo dot represented by an angle index and a sweep index at the
         * echoes to image look-up table
         *
         * @param angle_idx The angle index
         * @param sweep_idx The sweep index
         * @return std::pair<std::vector<cv::Point>::const_iterator,
         * std::vector<cv::Point>::const_iterator>
         */
        std::pair<size_t, size_t> getPixels(int angle_idx, int sweep_idx) const;
        /**
         * @brief Returns the echoes to image look-up table
         *
         * @return std::vector<cv::Point>
         */
        std::vector<cv::Point> getLUT();
        /**
         * Constructor that creates a LUT matrix based on radar echoes.
         * @brief Constructor.
         * @param num_angles Amount of samples contained in a radar revolution.
         * @param sweep_size Number of measurements inside a single sweep.
         * @param beam_width Angle between the 3dB half power points on the main lobe.
         * @param window_size The param will create a squared frame.
         */
        EchoToImageLUT(int num_angles, int sweep_size, float beam_width, int window_size);
        ~EchoToImageLUT() = default;

        void updateImage(cv::Mat& image, int angle, int echo_index, int echo) const;

        /**
         * Checks if provided configuration matches the LUT.
         *
         * @param num_angles Amount of sweeps contained in a radar revolution.
         * @param sweep_size Number of measurements inside a single sweep.
         * @param beam_width Angle between the 3dB half power points on the main lobe.
         * @param window_size The param will create a squared frame.
         * @return whether the LUT configuration matches the given parameters
         */
        bool hasMatchingConfiguration(int num_angles,
            int sweep_size,
            float beam_width,
            int window_size);

        void linearizeRawTable(RawTable const& table);

        /**
         * Draws image based on a vector representing the full rotation of the radar.
         *
         * @param world_echoes the vector representing a full radar rotation
         * @param radar_frame the frame where the radar will be drawn in
         */
        void drawImageFromEchoes(std::vector<uint8_t> const& world_echoes,
            cv::Mat& radar_frame);

        cv::Point fetchEntry(int angle, int echo_index) const;
    };
}

#endif