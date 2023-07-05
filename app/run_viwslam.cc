#include "utility/basetype.h"
#include "estimator/parameters.h"
#include "estimator/estimator.h"

string data_folder = "/workspace_18_04/dataset/urban39-pankyo/";
string config_file = "config/kaist/kaist_cam0_viwo_38-39.yaml";

vector<ImageData> vec_img_datas;
vector<ImuData> vec_imu_datas;
vector<WheelData> vec_wheel_datas;
Estimator estimator;
vector<double> vec_input_image_time;

void load_wheel_data(string path);
void load_image_data(string path);
void load_imu_data(string path);
void fusion_img_imu_wheel();

int main(int argc, char** argv)
{
    cout << "run_viwslam start" << endl;
    readParameters(config_file);
    estimator.setParameter();
    load_image_data(data_folder+"image_filenames.txt");
    load_imu_data(data_folder+"sensor_data/xsens_imu.csv");
    load_wheel_data(data_folder+"sensor_data/encoder.csv");
    fusion_img_imu_wheel();
    cout << "run_viwslam end" << endl;
    while(1);
    return 0;
}

void fusion_img_imu_wheel()
{
    std::ofstream ofs_kaist_pose;
    ofs_kaist_pose.open("output/kaist_pose.txt");
    int index_wheel = 1;
    int index_image = 0;
    int begin_i = 0;
    for(int i=begin_i; i<24000; i++)
    {
        // cout << fixed << setprecision(3) << "imu_" << i << " --> " << vec_imu_datas[i].timestamp << endl;
        estimator.inputIMU(vec_imu_datas[i].timestamp, vec_imu_datas[i].acc, vec_imu_datas[i].gyro);
        if(vec_wheel_datas[index_wheel].timestamp < vec_imu_datas[i].timestamp && index_wheel<vec_wheel_datas.size())
        {
            // cout << fixed << setprecision(3) << "wheel_" << index_wheel << " --> " << vec_wheel_datas[index_wheel].timestamp << endl;
            Eigen::Vector3d velocity(vec_wheel_datas[index_wheel].velocity,0,0);
            estimator.inputVEL(vec_wheel_datas[index_wheel].timestamp, velocity);
            index_wheel++;
        }
        if(vec_img_datas[index_image].timestamp < vec_imu_datas[i].timestamp && index_image<vec_img_datas.size())
        {
            // cout << fixed << setprecision(3) << "image_" << index_image << " --> " << vec_img_datas[index_image].timestamp << endl;
            cv::Mat gray_img = cv::imread(vec_img_datas[index_image].img_path, IMREAD_GRAYSCALE);
            estimator.inputImage(vec_img_datas[index_image].timestamp, gray_img);
            index_image++;
        }
        usleep(10000); // 10ms
    } 
    ofs_kaist_pose.close();
}

void load_image_data(string path)
{
    ifstream ifs_data(path, ifstream::in);
    string line;
    while (getline(ifs_data, line)) 
    {
        if(line.empty()) break;
        stringstream ss_data(line);
        std::vector<string> vec_strings;
        for(int i=0; i<1; i++)
        {
            string element;
            std::getline(ss_data, element, ',');
            if (element.empty()) break;
            vec_strings.push_back(element);
        }
        ImageData image_data;
        image_data.timestamp = stod(vec_strings[0])/1e9;
        image_data.img_path = data_folder + "image/" + vec_strings[0];
        vec_img_datas.push_back(image_data);
    }
    ifs_data.close();
    cout << "vec_img_datas.size() = " << vec_img_datas.size() << endl;
    // for(int i=1; i<vec_img_datas.size(); i++)
    // {
    //     cout << fixed << setprecision(3) << "img_" << i << " --> " << vec_img_datas[i].timestamp << ", " << vec_img_datas[i].img_path << endl;
    //     cv::Mat img0 = cv::imread(vec_img_datas[i].img_path, IMREAD_GRAYSCALE);
    //     cv::imshow("img0", img0);
    //     cv::waitKey(5);
    // }
}

void load_imu_data(string path)
{
    ifstream ifs_data(path, ifstream::in);
    string line;
    while (getline(ifs_data, line)) 
    {
        if(line.empty()) break;
        stringstream ss_data(line);
        std::vector<string> vec_strings;
        for(int i=0; i<17; i++)
        {
            string element;
            std::getline(ss_data, element, ',');
            // cout << i << " --> " << element << endl;
            if (element.empty()) break;
            vec_strings.push_back(element);
        }
        ImuData imu_data;
        imu_data.timestamp = stod(vec_strings[0])/1e9;
        imu_data.gyro[0] = stod(vec_strings[8]);
        imu_data.gyro[1] = stod(vec_strings[9]);
        imu_data.gyro[2] = stod(vec_strings[10]);
        imu_data.acc[0] = stod(vec_strings[11]);
        imu_data.acc[1] = stod(vec_strings[12]);
        imu_data.acc[2] = stod(vec_strings[13]);
        vec_imu_datas.push_back(imu_data);
    }
    ifs_data.close();
    cout << "vec_imu_datas.size() = " << vec_imu_datas.size() << endl;
    // for(int i=0; i<vec_imu_datas.size(); i++)
    // {
    //     cout << fixed << setprecision(3) << "imu_" << i << " --> " << vec_imu_datas[i].timestamp << ", " << vec_imu_datas[i].acc.transpose() << ", " << vec_imu_datas[i].gyro.transpose() << endl;
    // }
}

void load_wheel_data(string path)
{
    ifstream ifs_data(path, ifstream::in);
    string line;
    while (getline(ifs_data, line)) 
    {
        if(line.empty()) break;
        stringstream ss_data(line);
        std::vector<string> vec_strings;
        for(int i=0; i<3; i++)
        {
            string element;
            std::getline(ss_data, element, ',');
            if (element.empty()) break;
            vec_strings.push_back(element);
        }
        WheelData wheel_data;
        wheel_data.timestamp = stod(vec_strings[0])/1e9;
        wheel_data.left_count = stod(vec_strings[1]);
        wheel_data.right_count = stod(vec_strings[2]);
        vec_wheel_datas.push_back(wheel_data);
    }
    ifs_data.close();
    cout << "vec_wheel_datas.size() = " << vec_wheel_datas.size() << endl;
    // for(int i=0; i<vec_wheel_datas.size(); i++)
    // {
    //     cout << fixed << setprecision(3) << "wheel_" << i << " --> " << vec_wheel_datas[i].timestamp << ", " << vec_wheel_datas[i].left_count << ", " << vec_wheel_datas[i].right_count << endl;
    // }
    // calculate velocity from wheel encoder count
    double time_last = vec_wheel_datas[0].timestamp-0.001; // for avoid delta_t not to be zero
    double left_count_last = vec_wheel_datas[0].left_count;
    double right_count_last = vec_wheel_datas[0].right_count;
    for(int i=0; i<vec_wheel_datas.size(); i++)
    {
        double time_curr = vec_wheel_datas[i].timestamp;
        double left_count_curr = vec_wheel_datas[i].left_count;
        double right_count_curr = vec_wheel_datas[i].right_count;
        double delta_t = time_curr - time_last;
        // cout << fixed << setprecision(3) << "left_count_curr = " << left_count_curr << endl;
        // cout << fixed << setprecision(3) << "right_count_curr = " << right_count_curr << endl;
        // cout << fixed << setprecision(3) << "delta_t = " << delta_t << endl;
        vec_wheel_datas[i].left_speed = (left_count_curr-left_count_last)*0.623022*M_PI/4096.0/delta_t;
        vec_wheel_datas[i].right_speed = (right_count_curr-right_count_last)*0.622356*M_PI/4096.0/delta_t;
        vec_wheel_datas[i].velocity = (vec_wheel_datas[i].left_speed+vec_wheel_datas[i].right_speed)/2;
        vec_wheel_datas[i].timestamp = (time_curr+time_last)/2;
        left_count_last = left_count_curr;
        right_count_last = right_count_curr;
        time_last = time_curr;
    }
    // for(int i=0; i<vec_wheel_datas.size(); i++)
    // {
    //     cout << fixed << setprecision(3) << "wheel_" << i << " --> " << vec_wheel_datas[i].timestamp << ", " << vec_wheel_datas[i].velocity << endl;
    // }
}
