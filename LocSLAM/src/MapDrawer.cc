/**
* This file is part of LocSLAM.
*
* Copyright (C) 2020-2021 Lars Hammarstrand <lars.hammarstrand at chalmers dot se> (2021)
* For more information see <https://github.com/rulllars/SequentialLocalization>
*
* This file is a modified version of the file with the same name in ORB_SLAM2:
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* LocSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LocSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LocSLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace LocSLAM
{

    MapDrawer::MapDrawer(Map *pMap, Map *pFixedMap, const string &strSettingPath) : mpMap(pMap), mpFixedMap(pFixedMap),
                                                                                    mpCurrentKeyFrame(static_cast<KeyFrame *>(NULL))
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
        mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
        mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
        mPointSize = fSettings["Viewer.PointSize"];
        mCameraSize = fSettings["Viewer.CameraSize"];
        mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

        mvpKFs = mpMap->GetAllKeyFrames();
        sort(mvpKFs.begin(), mvpKFs.end(), KeyFrame::lTime);
    }

    void MapDrawer::DrawMapPoints()
    {
        unique_lock<recursive_mutex> lock(mpMap->mMutexMap);

        const vector<MapPoint *> &vpMPs = mpMap->GetAllMapPoints();
        //const vector<MapPoint *> &vpRefMPs = mpMap->GetReferenceMapPoints();
        const list<MapPoint *> &lpRefMPs = mpMap->GetLocalMapPoints();

        set<MapPoint *> spRefMPs(lpRefMPs.begin(), lpRefMPs.end());

        if (vpMPs.empty())
            return;

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        //glColor4f(0.6, 0.6, 0.6, 0.4);
        glColor4fv(vcStaticMapPoints);

        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
        {
            if (vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
                continue;
            cv::Mat pos = vpMPs[i]->GetWorldPos();
            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
            //cout << "Map point: " << pos.t() << endl;
        }
        glEnd();

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        //glColor4f(0.6, 0.6, 0.6, 0.4);
        glColor4fv(vcLocalMapPoints);

        for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++)
        {
            if ((*sit)->isBad())
                continue;
            cv::Mat pos = (*sit)->GetWorldPos();
            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
            //cout << "Reference map point: " << pos.t() << endl;
        }

        glEnd();
    }

    void MapDrawer::DrawFixedMapPoints()
    {
        unique_lock<recursive_mutex> lock(mpFixedMap->mMutexMap);

        const vector<MapPoint *> &vpMPs = mpFixedMap->GetAllMapPoints();

        if (vpMPs.empty())
            return;

        if (mpFixedMap->GetTransformation().empty())
            return;

        cv::Mat Tlf = mpFixedMap->GetTransformation();
        cv::Mat Rlf = Tlf.rowRange(0, 3).colRange(0, 3);
        cv::Mat tlf = Tlf.rowRange(0, 3).col(3);

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        //glColor3f(1.0, 0.3, 1.0);
        glColor4fv(vcFixedMapPoints);

        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
        {
            if (vpMPs[i]->isBad())
                continue;
            cv::Mat pos = Rlf * vpMPs[i]->GetWorldPos() + tlf;
            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
        }
        glEnd();
    }

    void MapDrawer::DrawLocalFixedMapPoints()
    {

        unique_lock<recursive_mutex> lock(mpMap->mMutexMap);

        const list<MapPoint *> &lpRefMPs = mpFixedMap->GetLocalMapPoints();

        set<MapPoint *> spRefMPs(lpRefMPs.begin(), lpRefMPs.end());

        if (mpFixedMap->GetTransformation().empty())
            return;

        cv::Mat Tlf = mpFixedMap->GetTransformation();
        cv::Mat Rlf = Tlf.rowRange(0, 3).colRange(0, 3);
        cv::Mat tlf = Tlf.rowRange(0, 3).col(3);

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        //glColor3f(1.0, 0.3, 1.0);
        glColor4fv(vcFixedMapPoints);

        for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++)
        {
            if ((*sit)->isBad())
                continue;
            cv::Mat pos = Rlf * (*sit)->GetWorldPos() + tlf;
            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
            //cout << "Reference map point: " << pos.t() << endl;
        }

        glEnd();
    }

    void MapDrawer::DrawLocalMapPoints()
    {

        unique_lock<recursive_mutex> lock(mpMap->mMutexMap);

        const list<MapPoint *> &lpRefMPs = mpMap->GetLocalMapPoints();

        set<MapPoint *> spRefMPs(lpRefMPs.begin(), lpRefMPs.end());

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor4fv(vcLocalMapPoints);

        for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++)
        {
            if ((*sit)->isBad())
                continue;
            cv::Mat pos = (*sit)->GetWorldPos();
            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
        }

        glEnd();
    }

    void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawPriors, const bool bDrawGraph, const bool bDrawPose, const bool bDrawOdo)
    {
        const float &w = mKeyFrameSize;
        const float h = w * 0.75;
        const float z = w * 0.6;
        const unsigned int nMaxKeyFrames = 300;

        UpdateKeyFrames();

        // Draw position
        glPointSize(3 * mPointSize);
        glBegin(GL_POINTS);
        //glColor3f(0.3f, 0.3f, 0.7f);
        glColor4fv(vcPose);

        for (size_t i = 0; i < mvpKFs.size(); i++)
        {
            KeyFrame *pKF = mvpKFs[i];
            cv::Mat pos = pKF->GetBodyPosition();

            glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
        }
        glEnd();

        if (bDrawKF)
        {
            for (size_t i = 0; i < mvpKFs.size(); i++)
            {
                KeyFrame *pKF = mvpKFs[i];
                cv::Mat Twc = pKF->GetCameraPoseInverse().t();

                glPushMatrix();

                glMultMatrixf(Twc.ptr<GLfloat>(0));

                glLineWidth(mKeyFrameLineWidth);
                //glColor3f(0.3f, 0.3f, 0.7f);
                glColor4fv(vcPose);
                glBegin(GL_LINES);
                glVertex3f(0, 0, 0);
                glVertex3f(w, h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, h, z);

                glVertex3f(w, h, z);
                glVertex3f(w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(-w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(w, h, z);

                glVertex3f(-w, -h, z);
                glVertex3f(w, -h, z);
                glEnd();

                glPopMatrix();
            }

            // Draw camera lever
            glLineWidth(mGraphLineWidth);
            //glColor3f(0.3f, 0.3f, 0.7f);
            glColor4fv(vcPose);
            glBegin(GL_LINES);

            for (size_t i = 0; i < mvpKFs.size(); i++)
            {
                KeyFrame *pKF = mvpKFs[i];
                cv::Mat pos = pKF->GetBodyPosition();
                cv::Mat pos2 = pKF->GetCameraCenter();

                glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
                glVertex3f(pos2.at<float>(0), pos2.at<float>(1), pos2.at<float>(2));
            }
            glEnd();
        }

        if (bDrawPose)
        {
            // Draw orientation
            glLineWidth(mGraphLineWidth);
            //glColor3f(0.3f, 0.3f, 0.7f);
            glColor4fv(vcPose);
            glBegin(GL_LINES);

            for (size_t i = 0; i < mvpKFs.size(); i++)
            {
                KeyFrame *pKF = mvpKFs[i];
                cv::Mat pos = pKF->GetBodyPosition();

                cv::Mat tb_x = cv::Mat::zeros(3, 1, CV_32F);
                tb_x.at<float>(0) = 1.0;

                cv::Mat pos_x = pKF->GetBodyPosition() + pKF->GetBodyRotation() * tb_x;

                glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
                glVertex3f(pos_x.at<float>(0), pos_x.at<float>(1), pos_x.at<float>(2));

                cv::Mat tb_z = cv::Mat::zeros(3, 1, CV_32F);
                tb_z.at<float>(2) = -1.0;

                cv::Mat pos_z = pKF->GetBodyPosition() + pKF->GetBodyRotation() * tb_z;

                glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
                glVertex3f(pos_z.at<float>(0), pos_z.at<float>(1), pos_z.at<float>(2));
            }
            glEnd();
        }

        if (bDrawOdo)
        {
            // Draw odometry orientation
            glLineWidth(mGraphLineWidth);
            glColor4fv(vcOdo);
            glBegin(GL_LINES);

            for (size_t i = 0; i < mvpKFs.size(); i++)
            {
                KeyFrame *pKF = mvpKFs[i];
                cv::Mat pos = pKF->GetBodyPosition();

                KeyFrame *pKFNext = pKF->GetNextKeyFrame();

                if (pKFNext != NULL || !pKFNext->GetBodyPosition().empty())
                {
                    cv::Mat predTbw = pKFNext->mOdometry.GetTransformation() * pKF->GetPose();
                    cv::Mat predTwb = predTbw.inv();
                    cv::Mat predPos = predTwb.rowRange(0, 3).col(3);
                    glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
                    glVertex3f(predPos.at<float>(0), predPos.at<float>(1), predPos.at<float>(2));
                }
            }
            glEnd();
        }

        if (bDrawPriors)
        {
            // Draw position
            glPointSize(3 * mPointSize);
            glBegin(GL_POINTS);
            glColor4fv(vcPosePrior);

            cv::Mat Tlf = mpFixedMap->GetTransformation();
            if (!Tlf.empty())
            {

                for (size_t i = 0; i < mvpFixedMapKFs.size(); i++)
                {
                    KeyFrame *pKF = mvpFixedMapKFs[i];

                    if (!pKF->isOutlier())
                    {
                        cv::Mat Tlb = Tlf * pKF->GetFixedMapPosePrior().inv();
                        cv::Mat pos = Tlb.rowRange(0, 3).col(3);

                        glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
                    }
                }
            }
            glEnd();

            // Draw position
            glPointSize(2 * mPointSize);
            glBegin(GL_POINTS);
            glColor4fv(vcPosePriorOutlier);

            if (!Tlf.empty())
            {

                for (size_t i = 0; i < mvpFixedMapKFs.size(); i++)
                {
                    KeyFrame *pKF = mvpFixedMapKFs[i];

                    if (pKF->isOutlier())
                    {
                        cv::Mat Tlb = Tlf * pKF->GetFixedMapPosePrior().inv();
                        cv::Mat pos = Tlb.rowRange(0, 3).col(3);

                        glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
                    }
                }
            }
            glEnd();

            // Draw position
            glPointSize(mGraphLineWidth);
            glBegin(GL_LINES);
            //glColor3f(1.0, 0.3, 1.0);
            glColor4fv(vcPosePrior);

            //cv::Mat Tlf = mpFixedMap->GetTransformation();
            if (!Tlf.empty())
            {

                for (size_t i = 0; i < mvpFixedMapKFs.size(); i++)
                {
                    KeyFrame *pKF = mvpFixedMapKFs[i];

                    if (!pKF->isOutlier())
                    {
                        cv::Mat Tlb = Tlf * pKF->GetFixedMapPosePrior().inv();
                        cv::Mat pos = pKF->GetBodyPosition(); //Tlb.rowRange(0, 3).col(3);

                        cv::Mat tb_x = cv::Mat::zeros(3, 1, CV_32F); //(cv::Mat_<float>(3,1)> << 1.0, 0.0, 0.0);
                        tb_x.at<float>(0) = 1.0;

                        cv::Mat pos_x = pos + Tlb.rowRange(0, 3).colRange(0, 3) * tb_x;

                        glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
                        glVertex3f(pos_x.at<float>(0), pos_x.at<float>(1), pos_x.at<float>(2));

                        cv::Mat tb_z = cv::Mat::zeros(3, 1, CV_32F); //(cv::Mat_<float>(3,1)> << 1.0, 0.0, 0.0);
                        tb_z.at<float>(2) = -1.0;

                        cv::Mat pos_z = pos + Tlb.rowRange(0, 3).colRange(0, 3) * tb_z;

                        glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
                        glVertex3f(pos_z.at<float>(0), pos_z.at<float>(1), pos_z.at<float>(2));
                    }
                }
            }

            glEnd();

            // Draw position
            glPointSize(mGraphLineWidth);
            glBegin(GL_LINES);
            //glColor3f(1.0, 0.3, 1.0);
            glColor4fv(vcPosePriorOutlier);

            if (!Tlf.empty())
            {

                for (size_t i = 0; i < mvpFixedMapKFs.size(); i++)
                {
                    KeyFrame *pKF = mvpFixedMapKFs[i];

                    if (pKF->isOutlier())
                    {
                        cv::Mat Tlb = Tlf * pKF->GetFixedMapPosePrior().inv();
                        cv::Mat pos = pKF->GetBodyPosition(); //Tlb.rowRange(0, 3).col(3);

                        cv::Mat tb_x = cv::Mat::zeros(3, 1, CV_32F); //(cv::Mat_<float>(3,1)> << 1.0, 0.0, 0.0);
                        tb_x.at<float>(0) = 1.0;

                        cv::Mat pos_x = pos + Tlb.rowRange(0, 3).colRange(0, 3) * tb_x;

                        glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
                        glVertex3f(pos_x.at<float>(0), pos_x.at<float>(1), pos_x.at<float>(2));

                        cv::Mat tb_z = cv::Mat::zeros(3, 1, CV_32F); //(cv::Mat_<float>(3,1)> << 1.0, 0.0, 0.0);
                        tb_z.at<float>(2) = -1.0;

                        cv::Mat pos_z = pos + Tlb.rowRange(0, 3).colRange(0, 3) * tb_z;

                        glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
                        glVertex3f(pos_z.at<float>(0), pos_z.at<float>(1), pos_z.at<float>(2));
                    }
                }
            }
            glEnd();
        }

        if (bDrawGraph)
        {
            glLineWidth(mGraphLineWidth);
            //glColor4f(0.3f, 0.7f, .3f, 0.6f);
            glColor4fv(vcGraph);
            glBegin(GL_LINES);

            for (size_t i = 0; i < mvpKFs.size(); i++)
            {
                // Covisibility Graph
                const vector<KeyFrame *> vCovKFs = mvpKFs[i]->GetCovisiblesByWeight(100);
                cv::Mat Ow = mvpKFs[i]->GetBodyPosition();
                if (!vCovKFs.empty())
                {
                    for (vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end(); vit != vend; vit++)
                    {
                        if ((*vit)->mnId < mvpKFs[i]->mnId)
                            continue;
                        cv::Mat Ow2 = (*vit)->GetBodyPosition();
                        glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                        glVertex3f(Ow2.at<float>(0), Ow2.at<float>(1), Ow2.at<float>(2));
                    }
                }

                // Spanning tree
                KeyFrame *pParent = mvpKFs[i]->GetParent();
                if (pParent)
                {
                    cv::Mat Owp = pParent->GetBodyPosition();
                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                    glVertex3f(Owp.at<float>(0), Owp.at<float>(1), Owp.at<float>(2));
                }

                // Loops
                set<KeyFrame *> sLoopKFs = mvpKFs[i]->GetLoopEdges();
                for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++)
                {
                    if ((*sit)->mnId < mvpKFs[i]->mnId)
                        continue;
                    cv::Mat Owl = (*sit)->GetBodyPosition();
                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                    glVertex3f(Owl.at<float>(0), Owl.at<float>(1), Owl.at<float>(2));
                }
            }

            glEnd();
        }

        /* if (bDrawOdo)
        {
            glLineWidth(2 * mGraphLineWidth);
            //glColor4f(1.0f, 0.0f, 0.0f, 0.6f);
            glColor4fv(vcOdoConnection);
            glBegin(GL_LINES);

            for (size_t i = 0; i < mvpKFs.size(); i++)
            {
                for (size_t i = 0; i < mvpKFs.size(); i++)
                {
                    // Next Key Frame
                    KeyFrame *pKFnext = mvpKFs[i]->GetNextKeyFrame();
                    cv::Mat Ow = mvpKFs[i]->GetBodyPosition();
                    if (pKFnext != NULL && !pKFnext->GetBodyPosition().empty())
                    {
                        cv::Mat Ow2 = pKFnext->GetBodyPosition();
                        glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                        glVertex3f(Ow2.at<float>(0), Ow2.at<float>(1), Ow2.at<float>(2));
                    }
                }
            }

            glEnd();
        } */
    }

    void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
    {
        const float &w = mCameraSize;
        const float h = w * 0.75;
        const float z = w * 0.6;

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        glLineWidth(mCameraLineWidth);
        //glColor3f(0.0f, 1.0f, 0.0f);
        glColor4fv(vcUpdatedCamera);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
    }

    void MapDrawer::DrawCurrentPredictedCamera(pangolin::OpenGlMatrix &Twc)
    {
        const float &w = mCameraSize;
        const float h = w * 0.75;
        const float z = w * 0.6;

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        glLineWidth(mCameraLineWidth);
        glColor4fv(vcPredictedCamera);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
    }

    void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
    {
        unique_lock<recursive_mutex> lock(mMutexCamera);
        mCameraPose = Tcw.clone();
    }

    void MapDrawer::SetPredictedCameraPose(const cv::Mat &Tcw)
    {
        unique_lock<recursive_mutex> lock(mMutexCamera);
        mPredictedCameraPose = Tcw.clone();
    }

    void MapDrawer::SetViewCameraPose(const cv::Mat &Tbw)
    {
        unique_lock<recursive_mutex> lock(mMutexCamera);

        mBodyPose = Tbw.clone();
    }

    const pangolin::OpenGlMatrix MapDrawer::CVMat2OpenGL(cv::Mat Twc)
    {
        pangolin::OpenGlMatrix M;

        if (!Twc.empty())
        {
            cv::Mat Rwc(3, 3, CV_32F);
            cv::Mat twc(3, 1, CV_32F);
            {
                unique_lock<recursive_mutex> lock(mMutexCamera);
                Rwc = Twc.rowRange(0, 3).colRange(0, 3).t();
                twc = -Rwc * Twc.rowRange(0, 3).col(3);
            }

            M.m[0] = Rwc.at<float>(0, 0);
            M.m[1] = Rwc.at<float>(1, 0);
            M.m[2] = Rwc.at<float>(2, 0);
            M.m[3] = 0.0;

            M.m[4] = Rwc.at<float>(0, 1);
            M.m[5] = Rwc.at<float>(1, 1);
            M.m[6] = Rwc.at<float>(2, 1);
            M.m[7] = 0.0;

            M.m[8] = Rwc.at<float>(0, 2);
            M.m[9] = Rwc.at<float>(1, 2);
            M.m[10] = Rwc.at<float>(2, 2);
            M.m[11] = 0.0;

            M.m[12] = twc.at<float>(0);
            M.m[13] = twc.at<float>(1);
            M.m[14] = twc.at<float>(2);
            M.m[15] = 1.0;
        }
        else
            M.SetIdentity();

        return M;
    }

    void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
    {
        if (!mCameraPose.empty())
        {
            cv::Mat Rwc(3, 3, CV_32F);
            cv::Mat twc(3, 1, CV_32F);
            {
                unique_lock<recursive_mutex> lock(mMutexCamera);
                Rwc = mCameraPose.rowRange(0, 3).colRange(0, 3).t();
                twc = -Rwc * mCameraPose.rowRange(0, 3).col(3);
            }

            M.m[0] = Rwc.at<float>(0, 0);
            M.m[1] = Rwc.at<float>(1, 0);
            M.m[2] = Rwc.at<float>(2, 0);
            M.m[3] = 0.0;

            M.m[4] = Rwc.at<float>(0, 1);
            M.m[5] = Rwc.at<float>(1, 1);
            M.m[6] = Rwc.at<float>(2, 1);
            M.m[7] = 0.0;

            M.m[8] = Rwc.at<float>(0, 2);
            M.m[9] = Rwc.at<float>(1, 2);
            M.m[10] = Rwc.at<float>(2, 2);
            M.m[11] = 0.0;

            M.m[12] = twc.at<float>(0);
            M.m[13] = twc.at<float>(1);
            M.m[14] = twc.at<float>(2);
            M.m[15] = 1.0;
        }
        else
            M.SetIdentity();
    }

    void MapDrawer::GetCurrentOpenGLPredictedCameraMatrix(pangolin::OpenGlMatrix &M)
    {
        if (!mPredictedCameraPose.empty())
        {
            cv::Mat Rwc(3, 3, CV_32F);
            cv::Mat twc(3, 1, CV_32F);
            {
                unique_lock<recursive_mutex> lock(mMutexCamera);
                Rwc = mPredictedCameraPose.rowRange(0, 3).colRange(0, 3).t();
                twc = -Rwc * mPredictedCameraPose.rowRange(0, 3).col(3);
            }

            M.m[0] = Rwc.at<float>(0, 0);
            M.m[1] = Rwc.at<float>(1, 0);
            M.m[2] = Rwc.at<float>(2, 0);
            M.m[3] = 0.0;

            M.m[4] = Rwc.at<float>(0, 1);
            M.m[5] = Rwc.at<float>(1, 1);
            M.m[6] = Rwc.at<float>(2, 1);
            M.m[7] = 0.0;

            M.m[8] = Rwc.at<float>(0, 2);
            M.m[9] = Rwc.at<float>(1, 2);
            M.m[10] = Rwc.at<float>(2, 2);
            M.m[11] = 0.0;

            M.m[12] = twc.at<float>(0);
            M.m[13] = twc.at<float>(1);
            M.m[14] = twc.at<float>(2);
            M.m[15] = 1.0;
        }
        else
            M.SetIdentity();
    }

    void MapDrawer::GetCurrentOpenGLBodyMatrix(pangolin::OpenGlMatrix &M)
    {
        if (!mBodyPose.empty())
        {
            cv::Mat Rwc(3, 3, CV_32F);
            cv::Mat twc(3, 1, CV_32F);
            {
                unique_lock<recursive_mutex> lock(mMutexCamera);
                Rwc = mBodyPose.rowRange(0, 3).colRange(0, 3).t();
                twc = -Rwc * mBodyPose.rowRange(0, 3).col(3);
            }

            M.m[0] = Rwc.at<float>(0, 0);
            M.m[1] = Rwc.at<float>(1, 0);
            M.m[2] = Rwc.at<float>(2, 0);
            M.m[3] = 0.0;

            M.m[4] = Rwc.at<float>(0, 1);
            M.m[5] = Rwc.at<float>(1, 1);
            M.m[6] = Rwc.at<float>(2, 1);
            M.m[7] = 0.0;

            M.m[8] = Rwc.at<float>(0, 2);
            M.m[9] = Rwc.at<float>(1, 2);
            M.m[10] = Rwc.at<float>(2, 2);
            M.m[11] = 0.0;

            M.m[12] = twc.at<float>(0);
            M.m[13] = twc.at<float>(1);
            M.m[14] = twc.at<float>(2);
            M.m[15] = 1.0;
        }
        else
            M.SetIdentity();
    }

    void MapDrawer::GetCurrentBodyMatrix(cv::Mat &Tbw)
    {
        Tbw = mBodyPose.clone();
    }

    void MapDrawer::UpdateKeyFrames()
    {
        const unsigned int nMaxKeyFrames = 300;

        if (mpMap)
        {
            long unsigned int nKeyFrames = mpMap->KeyFramesInMap();

            {
                unique_lock<recursive_mutex> lock(mpMap->mMutexMap);
                mvpKFs = mpMap->GetAllKeyFrames();
                sort(mvpKFs.begin(), mvpKFs.end(), KeyFrame::lTime);
            }

            /*
        if (nKeyFrames > nMaxKeyFrames)
        {
            mvpKFs.erase(mvpKFs.begin(), mvpKFs.begin() + nKeyFrames - nMaxKeyFrames);
        }
        */

            {
                unique_lock<recursive_mutex> lock(mpFixedMap->mMutexMap);
                mvpFixedMapKFs = mpFixedMap->GetAllKeyFrames();
                sort(mvpFixedMapKFs.begin(), mvpFixedMapKFs.end(), KeyFrame::lTime);
            }
        }
    }

    void MapDrawer::SetCurrentKeyFrame(KeyFrame *pCurrentKF)
    {
        mpCurrentKeyFrame = pCurrentKF;
    }

} // namespace LocSLAM
