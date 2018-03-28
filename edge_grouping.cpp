
#include "edge_grouping.h"
Mat  threshed;

int objects::minDistance(Point p1, Point p2, Point q1, Point q2) {
    int mi = 100000;
    // p1
    mi = min(mi, max(abs(p1.x - q1.x), abs(p1.y - q1.y)));
    mi = min(mi, max(abs(p1.x - q1.x), abs(p1.y - q2.y)));
    mi = min(mi, max(abs(p1.x - q2.x), abs(p1.y - q1.y)));
    mi = min(mi, max(abs(p1.x - q2.x), abs(p1.y - q2.y)));
    //p2
    mi = min(mi, max(abs(p2.x - q1.x), abs(p2.y - q1.y)));
    mi = min(mi, max(abs(p2.x - q1.x), abs(p2.y - q2.y)));
    mi = min(mi, max(abs(p2.x - q2.x), abs(p2.y - q1.y)));
    mi = min(mi, max(abs(p2.x - q2.x), abs(p2.y - q2.y)));
    //p1xp2y
    mi = min(mi, max(abs(p1.x - q1.x), abs(p2.y - q1.y)));
    mi = min(mi, max(abs(p1.x - q1.x), abs(p2.y - q2.y)));
    mi = min(mi, max(abs(p1.x - q2.x), abs(p2.y - q1.y)));
    mi = min(mi, max(abs(p1.x - q2.x), abs(p2.y - q2.y)));
    //p1yp2x
    mi = min(mi, max(abs(p2.x - q1.x), abs(p1.y - q1.y)));
    mi = min(mi, max(abs(p2.x - q1.x), abs(p1.y - q2.y)));
    mi = min(mi, max(abs(p2.x - q2.x), abs(p1.y - q1.y)));
    mi = min(mi, max(abs(p2.x - q2.x), abs(p1.y - q2.y)));
    return mi;
}

void objects::grouping(Mat &image, int j) {

    for (int e = 0; e < candidat.size(); e++) {
        if (e != j && candidat[e].positiongroup == 0)
            //~ if(1)
        {

            if (minDistance(candidat[j].origin, candidat[j].endpoint, candidat[e].origin, candidat[e].endpoint) < 5) {

                if (abs(candidat[j].apparition - candidat[e].apparition) < 50) {

                    if (candidat[j].positiongroup == 0) {
                        candidat[e].positiongroup = compteur;
                        candidat[j].positiongroup = compteur;
                        compteur++;
                        grouping(image, e);
                    }
                    else if (candidat[j].positiongroup > 0) {
                        candidat[e].positiongroup = candidat[j].positiongroup;
                        grouping(image, e);
                    }
                 //   line(map2_temp, candidat[j].centre, candidat[e].centre, Scalar(255, 0, 0));

                }

            }


        }

    }


}

void objects::extractObject(Mat &image, Mat & frame, int i,  Mat & map2) {
   // cvtColor(map2, map2_temp, CV_GRAY2RGB);
    vector<Rect> boxes;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

    if (!contours.empty() && !hierarchy.empty()) {
       
        int idx = 0;
        for (; idx >= 0; idx = hierarchy[idx][0]) {
            const vector<Point>& c = contours[idx];
            if (arcLength(c, false) > 20)
            {
       
                boxes.push_back(boundingRect(c));
            }
        }
    }


    if (1) {

       // Mat img(frame.size(), CV_8UC3);
       // img = Scalar::all(0);

        int d = 0;
        for (int b = 0; b < boxes.size(); b++) {
            bool found = false;
            Point2f sample;
            sample.x = (float) (boxes[b].x + boxes[b].width / 2);
            sample.y = (float) (boxes[b].y + boxes[b].height / 2);

            for (int j = 0; j < candidat.size(); j++) {
                candidat[j].skip = false;
                candidat[j].positiongroup = 0;
                //~ if(blob->minx-candidat[j].origin.x<5 && blob->miny-candidat[j].origin.y<5 && blob->maxx-candidat[j].endpoint.x<5 && blob->maxy-candidat[j].endpoint.y<5 )
                if (abs(sample.x - candidat[j].centre.x) < 5 && abs(sample.y - candidat[j].centre.y) < 5)
 {

                    candidat[j].centre.x = sample.x;
                    candidat[j].centre.y = sample.y;
                    candidat[j].origin.x = boxes[b].x;
                    candidat[j].origin.y = boxes[b].y;
                    candidat[j].endpoint.x = boxes[b].x + boxes[b].width;
                    candidat[j].endpoint.y = boxes[b].y + boxes[b].height;

                    candidat[j].activeness = 40;
                    candidat[j].active = true;
                    found = true;
                    break;
                }

             //  rectangle(map2_temp, Rect(candidat[j].origin, candidat[j].endpoint), Scalar(0, 255, 255), 1);
             //   rectangle(threshed, Rect(candidat[j].origin, candidat[j].endpoint), Scalar(0, 255, 255), 1);

            }
            if (!found) {
                object obj;
                obj.positiongroup = 0;
                obj.centre.x = sample.x;
                obj.centre.y = sample.y;
                obj.origin.x = boxes[b].x;
                obj.origin.y = boxes[b].y;
                obj.endpoint.x = boxes[b].x + boxes[b].width;
                obj.endpoint.y = boxes[b].y + boxes[b].height;
                obj.lifetime = 0;
                obj.activeness = 40;
                obj.active = true;
                obj.apparition = i;
                candidat.push_back(obj);
            }

            //~ free(blob);

            d++;
        }

        compteur = 1;
        for (int j = 0; j < candidat.size(); j++) {
            if (candidat[j].skip) continue;
            grouping(map2, j);
            if (candidat[j].lifetime < 150);

      
           // rectangle(map2, Rect(candidat[j].origin, candidat[j].endpoint), Scalar(255, 0, 0));


        }
        for (int j = 0; j < candidat.size(); j++) {

            candidat[j].lifetime++;

            candidat[j].proc = false;
            if (candidat[j].activeness > 0)
                candidat[j].activeness--;
            if (candidat[j].activeness <= 0)
                candidat.erase(candidat.begin() + j);
           
            if (candidat[j].lifetime > 20 && candidat[j].positiongroup == 0) {
                //	cout << " efefefefef" << endl;
                candidat[j].skip = true;
                AO aband;
                aband.lifetime = 0;
                aband.activeness = 20;
                aband.abandoness = 0;
                aband.centre.x = candidat[j].centre.x;
                aband.centre.y = candidat[j].centre.y;
                aband.origin.x = candidat[j].origin.x;
                aband.origin.y = candidat[j].origin.y;
                aband.endpoint.x = candidat[j].endpoint.x;
                aband.endpoint.y = candidat[j].endpoint.y;
                aband.update = true;
                this->abandonnes.push_back(aband);

            }


        }


        for (int j = 0; j < candidat.size(); j++)
 {
            int label = candidat[j].positiongroup;
            Rect obje;
            int r = 0;
            if (candidat[j].positiongroup != 0 && !candidat[j].proc)
                for (int e = 0; e < candidat.size(); e++)
 {

                    if (label == candidat[e].positiongroup && e != j)//&& candidat[e].positiongroup!=0 )
                    {
                        candidat[e].proc = true;
                  
                        if (r == 0)

                            obje = Rect(candidat[j].origin, candidat[j].endpoint) | Rect(candidat[e].origin, candidat[e].endpoint);
                        else
                            obje = Rect(candidat[e].origin, candidat[e].endpoint) | obje;

                        r++;
                    }

                }

          //  rectangle(frame, obje, Scalar(255, 255, 255));
            //rectangle(map2_temp, obje, Scalar(255, 255, 255));
            Point centre(obje.x + obje.width / 2, obje.y + obje.height / 2);
            //~ circle(map2,  centre, 4, Scalar(255,255,0), 4);
            bool found = false;
            if (centre.x != 0 && centre.y != 0) {
                if (this->abandonnes.size() == 0) {

                    AO aband;
                    aband.lifetime = 0;
                    aband.activeness = 20;
                    aband.abandoness = 0;
                    aband.centre.x = centre.x;
                    aband.centre.y = centre.y;
                    aband.origin.x = obje.x;
                    aband.origin.y = obje.y;
                    aband.endpoint.x = obje.x + obje.width;
                    aband.endpoint.y = obje.y + obje.height;
                    aband.update = true;
                    aband.boxarea = obje.width * obje.height;
                    this->abandonnes.push_back(aband);
                    //~ cout<<"ddd"<<endl;
                } else
                    for (int j = 0; j<this->abandonnes.size(); j++)
 {

                        if (abs(centre.x - this->abandonnes[j].centre.x) < 20 && abs(centre.y - this->abandonnes[j].centre.y) < 20)
 {
                            found = true;
                            if (this->abandonnes[j].activeness <= 20)
                                this->abandonnes[j].activeness++;
                            this->abandonnes[j].centre.x = centre.x;
                            this->abandonnes[j].centre.y = centre.y;
                            this->abandonnes[j].origin.x = obje.x;
                            this->abandonnes[j].origin.y = obje.y;
                            this->abandonnes[j].endpoint.x = obje.x + obje.width;
                            this->abandonnes[j].endpoint.y = obje.y + obje.height;
                            //~ break;

                        }
                    }
                if (!found) {

                    AO aband;
                    aband.lifetime = 0;
                    aband.activeness = 20;
                    aband.centre.x = centre.x;
                    aband.centre.y = centre.y;
                    aband.origin.x = obje.x;
                    aband.origin.y = obje.y;
                    aband.endpoint.x = obje.x + obje.width;
                    aband.endpoint.y = obje.y + obje.height;
                    aband.boxarea = obje.width * obje.height;
                    aband.abandoness = 0;

                    this->abandonnes.push_back(aband);

                }

            }
        }
        for (int j = 0; j<this->abandonnes.size(); j++)
 {

            this->abandonnes[j].lifetime++;

         
            if (this->abandonnes[j].activeness > 0)
                this->abandonnes[j].activeness--;

            if (this->abandonnes[j].activeness <= 0)
                this->abandonnes.erase(this->abandonnes.begin() + j);

        }

    }
}


