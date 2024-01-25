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

        void setup(int sweep_size, int window_size, int num_angles, float beam_width);

    public:
        EchoToImageLUT() = delete;

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

        void updateImage(cv::Mat& image,
            int angle,
            int echo_index,
            int echo,
            bool force_write = false) const;

        void linearizeRawTable(RawTable const& table);

        cv::Point fetchEntry(int angle, int echo_index) const;
    };
}

#endif