#include <event_simulator/Player.h>
#include <fstream>
#include <string>
#include <iostream>


int main(void)
{
    int x_res = 640; int y_res = 480;
    int wait_time_ms = 0;
    int C_pos = 20, C_neg = 20, num_inter_frames = 10;
    int div_factor = 10;
    int C_offset = 10;
    
    std::shared_ptr<DenseOpticalFlowCalculator> ofc_dis_low = std::make_shared<DISOpticalFlowCalculator>(DISOpticalFlowQuality::LOW);
    std::shared_ptr<SparseOpticalFlowCalculator> ofc_lk = std::make_shared<LKOpticalFlowCalculator>();
    std::shared_ptr<EventSimulator> bdf = std::make_shared<BasicDifferenceEventSimulator>(C_pos, C_neg);
    std::shared_ptr<EventSimulator> dsmes =  std::make_shared<DenseInterpolatedEventSimulator>(ofc_dis_low, num_inter_frames, C_pos / div_factor, C_neg / div_factor);
    std::shared_ptr<EventSimulator> ssmes = std::make_shared<SparseInterpolatedFrameEventSimulator>(ofc_lk, num_inter_frames, C_pos / 2, C_neg / 2);
    std::shared_ptr<EventSimulator> sdies = std::make_shared<SparseInterpolatedEventSimulator>(ofc_lk, num_inter_frames, C_pos, C_neg, C_pos + C_offset, C_neg + C_offset);
    
    // std::string video_path =  "../res/videos/car.mp4"; 
    // OpenCVPlayer cv_player = OpenCVPlayer(dsmes, wait_time_ms);
    // cv_player.play(video_path);

    VideoStreamer streamer = VideoStreamer(sdies);
    streamer.stream(0);

    return 0;


    /*
    std::string video_path = "../res/videos/car.mp4"; // or use  "../res/videos/ball.avi"; 
                                                      //
    std::string video_path = "/data/UniCloud/teaching/thesis/simulating_event-based_cameras/submission/code/traditional/cpp_event_sim/res/videos/car.mp4"; // or use  "../res/videos/ball.avi"; 
    std::shared_ptr<DenseOpticalFlowCalculator> ofc_dis_low = std::make_shared<DISOpticalFlowCalculator>(DISOpticalFlowQuality::LOW);
    // std::shared_ptr<DenseOpticalFlowCalculator> ofc_dis_med = std::make_shared<DISOpticalFlowCalculator>(DISOpticalFlowQuality::MEDIUM);
    std::shared_ptr<DenseOpticalFlowCalculator> ofc_dis_ex = std::make_shared<DISOpticalFlowCalculator>(DISOpticalFlowQuality::EXTREME);
    
    std::shared_ptr<DenseOpticalFlowCalculator> ofc_farne = std::make_shared<FarnebackFlowCalculator>();
    //std::shared_ptr<DenseOpticalFlowCalculator> ofc_cfarne = std::make_shared<CudaFarnebackFlowCalculator>();

    std::shared_ptr<SparseOpticalFlowCalculator> ofc_lk = std::make_shared<LKOpticalFlowCalculator>();
    //std::shared_ptr<SparseOpticalFlowCalculator> ofc_clk = std::make_shared<CudaLKOpticalFlowCalculator>();
    
    int C_pos = 20, C_neg = 20, num_inter_frames = 10;
    int div_factor = 10;
    int C_offset = 10;
    
    std::vector<std::shared_ptr<VideoRenderer>> renderers = {
        std::make_shared<BasicRenderer>(),
        std::make_shared<BasicDifferenceEventRenderer>(C_pos, C_neg),

        std::make_shared<DenseInterpolatedEventRenderer>(ofc_dis_low, num_inter_frames, C_pos / div_factor, C_neg / div_factor),
        // std::make_shared<DenseInterpolatedEventRenderer>(ofc_dis_med, num_inter_frames, C_pos / div_factor, C_neg / div_factor),
        std::make_shared<DenseInterpolatedEventRenderer>(ofc_dis_ex, num_inter_frames, C_pos / div_factor, C_neg / div_factor),

        //std::make_shared<DenseInterpolatedEventRenderer>(ofc_cfarne, num_inter_frames, C_pos / div_factor, C_neg / div_factor),
        std::make_shared<DenseInterpolatedEventRenderer>(ofc_farne, num_inter_frames, C_pos / div_factor, C_neg / div_factor),
      
        std::make_shared<DenseOpticalFlowRenderer>(ofc_dis_low),
        std::make_shared<DenseOpticalFlowRenderer>(ofc_dis_ex),
        std::make_shared<DenseOpticalFlowRenderer>(ofc_farne),

        std::make_shared<SparseInterpolatedFrameEventRenderer>(ofc_lk, num_inter_frames, C_pos / 2, C_neg / 2),
        //std::make_shared<SparseInterpolatedFrameEventRenderer>(ofc_clk, num_inter_frames, C_pos / 2, C_neg / 2),

        std::make_shared<SparseInterpolatedEventRenderer>(ofc_lk, num_inter_frames, C_pos, C_neg, C_pos + C_offset, C_neg + C_offset),
        //std::make_shared<SparseInterpolatedEventRenderer>(ofc_clk, num_inter_frames, C_pos, C_neg, C_pos + C_offset, C_neg + C_offset),
    };
    
    std::cout << "Starting to load video: " << video_path << " in camera" << std::endl;

    OpenCVPlayer cv_player = OpenCVPlayer(renderers[0], 0);
    // set an optional region of intrest
    // cv_player.roi = Rect(300, 120, 200, 200);

    // Code for rendering single frames for all renderers
    cv_player.load(video_path);
    for (auto renderer : renderers)
    {
      cv_player.setRenderer(renderer);
      cv_player.renderSingleFrame(video_path, 3);
    }
    // Code for timing the renderers
    // std::ofstream out("../res/times_640_480_ball.txt");
    //  for (auto renderer : renderers)
    // {
    //     cv_player.set_renderer(renderer);
    //     double frametime = cv_player.timed_play(1);
    //     std::string line = renderer->get_name() + " & " + std::to_string(frametime) + "ms \\\\ \n";
    //     std::cout << line;
    //     out << line;
    // }
    // out.close();

    return 0;
    */
}
