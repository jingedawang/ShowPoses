#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <pangolin/pangolin.h>
#include <opencv/cv.hpp>
#include "Converter.h"

using namespace std;

const float cameraSize = 3.0f;
const float cameraLineWidth = 3.0f;

void DrawCurrentCamera(vector<cv::Mat> showPoses);

int main() {

    // You should modify the absolute path below.
    string posePath = "/home/wjg/projects/ShowPoses/camera_pose2.txt";
    ifstream poseStream(posePath);
    vector<cv::Mat> poses;

    if (!poseStream.is_open())
    {
        cout << "Can not open file!" << endl;
        return 1;
    }

    // Read all poses.
    while (!poseStream.eof())
    {
        double _timestamp, x, y, z, qw, qx, qy, qz;
        poseStream >> _timestamp >> x >> y >> z >> qw >> qx >> qy >> qz;
        if (poseStream.eof())
        {
            break;
        }
        Eigen::Quaterniond q(qw, qx, qy, qz);
        Eigen::Matrix3d Rwc =  q.normalized().toRotationMatrix();
        Eigen::Quaterniond q_validation(Rwc);
        Eigen::Vector3d t(x, y, z);
        Eigen::Matrix4d Twc;
        Twc.topLeftCorner(3, 3) = Rwc;
        Twc.topRightCorner(3, 1) = t;
        Twc.bottomLeftCorner(1, 3).setZero();
        Twc(3, 3) = 1.0;
        cv::Mat Tcw = Converter::toCvMat(Twc).inv();
        poses.push_back(Tcw);
    }
    poseStream.close();

    // Initialize pangolin
    pangolin::CreateWindowAndBind("Show Poses",1024,768);
    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);
    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);

    float viewpointF = 500.0f;
    float viewpointX = 0.0f;
    float viewpointY = 40.0f;
    float viewpointZ = 20.0f;

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024,768,viewpointF,viewpointF,512,389,0.1,1000),
            pangolin::ModelViewLookAt(viewpointX,viewpointY,viewpointZ, 0,-10,0,0.0,-1.0, 0.0)
    );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    bool bFollow = true;

    // Every time, show one more pose.
    vector<cv::Mat> showPoses;
    for (const cv::Mat &pose : poses)
    {
        showPoses.push_back(pose);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        Twc = Converter::GetCurrentOpenGLCameraMatrix(pose);

        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(viewpointX,viewpointY,viewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        DrawCurrentCamera(showPoses);

        pangolin::FinishFrame();
        cv::waitKey(200);
    }

}

void DrawCurrentCamera(vector<cv::Mat> showPoses)
{
    const float &w = cameraSize;
    const float h = w*0.75f;
    const float z = w*0.6f;

    for (const cv::Mat pose : showPoses)
    {
        glPushMatrix();
        // Use Twc_t instead of Twc because OpenGL use right multiplication
        cv::Mat Twc_t = pose.inv().t();
        glMultMatrixf(Twc_t.ptr<GLfloat>(0));

        glLineWidth(cameraLineWidth);
        glColor3f(0.0f,1.0f,0.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }


}