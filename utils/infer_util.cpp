#include <torch/script.h>
#include <torch/torch.h>
#include <opencv2/opencv.hpp>

#include "infer_util.h"

at::Tensor PTModel::matToTensor(const cv::Mat& mat){
    cv::Mat img;
    mat.convertTo(img, CV_32F, 1.0/255);
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    cv::resize(img, img, cv::Size(img_h, img_w));

    auto tensor = torch::from_blob(
        img.data,
        {1, img.rows, img.cols, 3},
        torch::kFloat32
    );

    tensor = tensor.permute({0, 3, 1, 2});
    return tensor.clone();
}

void PTModel::pushInput(at::Tensor input){
    inputs.push_back(input);
}

void PTModel::clearInputs(){
    inputs.clear();
}

void PTModel::run(){
    result = module.forward(inputs).toTuple();
}

at::Tensor PTModel::getOutput(int output_index){
    return result->elements()[output_index].toTensor();
}

void PTModel::drawBox(cv::Mat& img, const cv::Rect& box, const cv::Scalar& color){
    cv::rectangle(img, box, color, 2);
}

void PTModel::resizeBox(cv::Rect& box, cv::Size from, cv::Size to){
    float gain_w = (float)to.width / from.width;
    float gain_h = (float)to.height / from.height;
    box.x = std::round(box.x * gain_w);
    box.y = std::round(box.y * gain_h);
    box.width = std::round(box.width * gain_w);
    box.height = std::round(box.height * gain_h);
}

void PTModel::predProcess(){
    // std::cout << "result size : " << result->elements().size() << std::endl;
    auto output = getOutput(0);
    // std::cout << output <<std::endl;
    auto pred = output.accessor<float, 3>();

    for(int i=0; i<pred.size(1); i++){
        float x_center = pred[0][i][0];
        float y_center = pred[0][i][1];
        float width =    pred[0][i][2];
        float height =   pred[0][i][3];
        float obj_conf = pred[0][i][4];

        /***************************************
         *****   for multiple class case   *****
         ***************************************/
        // float max_score = 0.0f;
        // int class_id = -1;
        // for(int j=5; j<6; j++){ // 6 = (# of class) + (xywh : 4) + (conf : 1)
        //     float class_conf = pred[0][i][j];
        //     if(class_conf > max_score){
        //         max_score = class_conf;
        //         class_id = j-5;
        //     }
        // }
        // float total_conf = obj_conf * max_score;
        // if(total_conf > this->conf){
        //   /* process : detected class */
        // }

        if(obj_conf > this->conf){
            float x = x_center - width / 2.0f;
            float y = y_center - height / 2.0f;
            cv::Rect rect(x, y, width, height);
            rects.push_back(rect);
        }

    }
    
}

void PTModel::drawPredBoxes(cv::Mat& img){

    std::vector<cv::Scalar> colors = {(0, 255, 0)};
    int index = 0;
    for(auto rect : rects){
        // drawBox(img, rect, colors[index%colors.size()]);
        // index++;
        drawBox(img, rect, colors[0]);
    }
}