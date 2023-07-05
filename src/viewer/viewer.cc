#include "viewer.h"

Viewer::Viewer()
{
    m_viewer_thread = new thread(&Viewer::ViewerLoop, this);
}

void Viewer::ViewerLoop()
{
    pangolin::CreateWindowAndBind("Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, m_config.view_point_f, m_config.view_point_f, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(m_config.view_point_x, m_config.view_point_y, m_config.view_point_z, 
                                                             m_config.view_center_x, m_config.view_center_y, m_config.view_center_z, 
                                                             m_config.view_up_x, m_config.view_up_y, m_config.view_up_z)
    );
    pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));
	int UI_WIDTH = 175;
	pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
    pangolin::Var<bool> is_show_image("menu.show_image", true, true);  
    pangolin::Var<bool> is_show_odo_pose("menu.show_odo_pose", true, true); 
    pangolin::Var<bool> is_show_loop_pose("menu.show_loop_pose", true, true); 
    pangolin::Var<bool> is_show_map_pose("menu.show_map_pose", true, true); 
    pangolin::Var<bool> is_show_pointcloud("menu.show_pointcloud", true, true); 
    pangolin::Var<int> grid_scale("menu.Grid Size (m)", 100, 1, 200);
    pangolin::Var<double> image_scale("menu.image_scale", 1.0, 0.1, 2);
    pangolin::Var<bool> show_grid("menu.Show Grid", false, true);
    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);
        pangolin::glDrawAxis(3);
        // Draw grid.
        if (show_grid.Get()) 
        {
            glColor3f(0.3f, 0.3f, 0.3f);
            pangolin::glDraw_z0(grid_scale, 1000);
        }
        // for mutex
        {
            unique_lock<mutex> lock(m_data_mutex);
            if(is_show_image)
            {
                ShowImage(m_image, image_scale);
            }
            if(is_show_odo_pose)
            {
                ShowPoses(mvec_odo_pose);
            }
            if(is_show_loop_pose)
            {
                ShowLoopPoses(mvec_loop_pose);
            }
            if(is_show_map_pose)
            {
                ShowOfflineMapPoses(mvec_offlinemap_pose);
            }
            if(is_show_pointcloud)
            {
                ShowPointcloud(m_pointCloud);
            }
        }
        pangolin::FinishFrame();
    }
}

void Viewer::UpdateOdoPose(Isometry3d& pos)
{
    unique_lock<mutex> lock(m_data_mutex);
    mvec_odo_pose.push_back(pos);
}

void Viewer::UpdateLoopPose(vector<Isometry3d>& vec_loop_pose)
{
    unique_lock<mutex> lock(m_data_mutex);
    mvec_loop_pose = vec_loop_pose;
}

void Viewer::UpdateOfflineMapPose(Isometry3d& pos)
{
    unique_lock<mutex> lock(m_data_mutex);
    mvec_offlinemap_pose.push_back(pos);
}

void Viewer::ShowOfflineMapPoses(vector<Isometry3d>& vec_pos)
{
    if(vec_pos.size() == 0) return;
    // draw every point
    for(int i=0; i<vec_pos.size(); i++)
    {
        glPointSize(2.0f);
        glBegin(GL_POINTS);   
        glColor3f(0.0,1.0,0.0); // green
        Vector3d p = vec_pos[i].translation();
        glVertex3d(p[0], p[1], p[2]);
        glEnd(); 
    }
}

void Viewer::ShowLoopPoses(vector<Isometry3d>& vec_pos)
{
    if(vec_pos.size() == 0) return;
    // draw every point
    for(int i=0; i<vec_pos.size(); i++)
    {
        glPointSize(2.0f);
        glBegin(GL_POINTS);   
        glColor3f(1.0,0.0,0.0); // red
        Vector3d p = vec_pos[i].translation();
        glVertex3d(p[0], p[1], p[2]);
        glEnd(); 
    }
}

void Viewer::ShowPoses(vector<Isometry3d>& vec_pos)
{
    if(vec_pos.size() == 0) return;
    // draw every point
    for(int i=0; i<vec_pos.size(); i++)
    {
        glPointSize(2.0f);
        glBegin(GL_POINTS);   
        glColor3f(0.0,0.0,1.0); // blue
        Vector3d p = vec_pos[i].translation();
        glVertex3d(p[0], p[1], p[2]);
        glEnd(); 
    }
    // draw coordinate axis for the latest pose
    Vector3d Ow = vec_pos[vec_pos.size()-1].translation();
    Vector3d Xw = vec_pos[vec_pos.size()-1] * (3 * Vector3d(1, 0, 0)); 
    Vector3d Yw = vec_pos[vec_pos.size()-1] * (3 * Vector3d(0, 1, 0));
    Vector3d Zw = vec_pos[vec_pos.size()-1] * (3 * Vector3d(0, 0, 1));
    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0); // red
    glVertex3d(Ow[0], Ow[1], Ow[2]);
    glVertex3d(Xw[0], Xw[1], Xw[2]);
    glColor3f(0.0, 1.0, 0.0); // green
    glVertex3d(Ow[0], Ow[1], Ow[2]);
    glVertex3d(Yw[0], Yw[1], Yw[2]);
    glColor3f(0.0, 0.0, 1.0); // blue
    glVertex3d(Ow[0], Ow[1], Ow[2]);
    glVertex3d(Zw[0], Zw[1], Zw[2]);
    glEnd();
}

void Viewer::ShowPointcloud(vector<Vector3d> point_cloud)
{
    // draw every point
    for(int i=0; i<point_cloud.size(); i++)
    {
        glPointSize(5.0f);
        glBegin(GL_POINTS);   
        glColor3f(0.0,0.0,1.0); // blue
        Vector3d p = point_cloud[i];
        glVertex3d(p[0], p[1], p[2]);
        glEnd(); 
    }
}

void Viewer::UpdateImage(cv::Mat& img)
{
    unique_lock<mutex> lock(m_data_mutex);
    img.copyTo(m_image);
}

void Viewer::UpdatePointCloud(vector<Vector3d> point_cloud)
{
    unique_lock<mutex> lock(m_data_mutex);
    m_pointCloud = point_cloud;
}

void Viewer::ShowImage(cv::Mat image, double scale)
{
    if(!image.empty())
    {
        cv::resize(image,image,cv::Size(image.cols*scale, image.rows*scale));
        cv::imshow("image", image);
        cv::waitKey(5);
    }
}