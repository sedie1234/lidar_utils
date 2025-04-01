#ifndef INFER_UTIL_H
#define INFER_UTIL_H

#include <torch/script.h>
#include <torch/torch.h>
#include <opencv2/opencv.hpp>

class PTModel{ // yolov5, input shape = {1, 3, 640, 640}, output shape = {1, 25200, 6}
public:
    PTModel(std::string model_name){
        try{
            module = torch::jit::load(model_name);
        }catch (const c10::Error& e){
            std::cerr << "error loading the model : " << model_name << std::endl;
        }
        img_h = 640;
        img_w = 640;
        conf  = 0.25;
    };

    at::Tensor matToTensor(const cv::Mat& mat);
    void pushInput(at::Tensor input);
    void clearInputs();
    void run();
    at::Tensor getOutput(int output_index);
    void drawBox(cv::Mat& img, const cv::Rect& box, const cv::Scalar& color);
    void drawPredBoxes(cv::Mat& img);
    void resizeBox(cv::Rect& box, cv::Size from, cv::Size to);
    void predProcess();

// private:
    torch::jit::script::Module module;
    std::vector<torch::jit::IValue> inputs;
    c10::intrusive_ptr<c10::ivalue::Tuple> result;
    
    int img_h;
    int img_w;
    float conf;

    std::vector<cv::Rect> rects;
    
};

class OnnxModel{

};


#endif // INFER_UTIL_H