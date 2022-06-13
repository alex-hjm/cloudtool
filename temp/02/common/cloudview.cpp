#include "cloudview.h"
CloudView::CloudView(QWidget *parent) :QVTKOpenGLNativeWidget(parent),
    showCloudIdEnable(true)
{
    m_ren = vtkSmartPointer<vtkRenderer>::New();
    m_renWnd = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    axes = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
}

void CloudView::init()
{
    m_renWnd->AddRenderer(m_ren);
    viewer.reset(new pcl::visualization::PCLVisualizer(m_ren,m_renWnd,"viewer", false));
    this->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(this->GetInteractor(),this->GetRenderWindow());
    viewer->setBackgroundColor(150.0 / 255.0, 150.0 / 255.0, 150.0 / 255.0);
    this->showCloudId("CloudTool");
    this->showAxes();
    m_renWnd->Render();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//update cloud
void CloudView::updateCloud(const CloudXYZRGBN::Ptr &cloud,const string &id)
{
    if(!viewer->contains(id))
        viewer->addPointCloud<PointXYZRGBN>(cloud,id);
    else{
        pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBN>rgb(cloud);
        viewer->updatePointCloud(cloud,rgb,id);
    }
    m_renWnd->Render();
}

void CloudView::updateCloud(const CloudXYZRGB::Ptr &cloud,const string &id)
{
    if(!viewer->contains(id))
        viewer->addPointCloud<PointXYZRGB>(cloud,id);
    else {
        pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGB>rgb(cloud);
        viewer->updatePointCloud(cloud,rgb,id);
    }
    m_renWnd->Render();
}

void CloudView::updateCloud(const CloudXYZI::Ptr &cloud,const string &id)
{
    if(!viewer->contains(id))
        viewer->addPointCloud<PointXYZI>(cloud,id);
    else {
        pcl::visualization::PointCloudColorHandlerRGBField<PointXYZI>rgb(cloud);
        viewer->updatePointCloud(cloud,rgb,id);
    }
    m_renWnd->Render();
}

void CloudView::updateCloud(const CloudXYZL::Ptr &cloud,const string &id)
{
    if(!viewer->contains(id))
        viewer->addPointCloud<PointXYZL>(cloud,id);
    else {
        pcl::visualization::PointCloudColorHandlerRGBField<PointXYZL>rgb(cloud);
        viewer->updatePointCloud(cloud,rgb,id);
    }
    m_renWnd->Render();
}

void CloudView::updateCloud(const CloudXYZRGBA::Ptr &cloud,const string &id)
{
    if(!viewer->contains(id))
        viewer->addPointCloud<PointXYZRGBA>(cloud,id);
    else {
        pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBA>rgb(cloud);
        viewer->updatePointCloud(cloud,rgb,id);
    }
    m_renWnd->Render();
}

void CloudView::updateCorrespondences(const CloudXYZRGBN::Ptr &source, const CloudXYZRGBN::Ptr &target,
                                      const CorrespondencesPtr &correspondences, const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addCorrespondences<PointXYZRGBN>(source,target,*correspondences,id);
    else {
         viewer->updateCorrespondences<PointXYZRGBN>(source,target,*correspondences,id);
    }
    m_renWnd->Render();
}
///////////////////////////////////////////////////////////////////////////////////////////////////
//update mesh
void CloudView::updateMesh(const PolygonMesh::Ptr &polymesh, const string &id)
{
    if(!viewer->contains(id))
        viewer->addPolygonMesh(*polymesh,id);
    else
        viewer->updatePolygonMesh(*polymesh,id);
    m_renWnd->Render();
}

void CloudView::updateMeshLine(const PolygonMesh::Ptr &polymesh, const string &id)
{
    if(!viewer->contains(id))
        viewer->addPolylineFromPolygonMesh(*polymesh,id);
    else {
        viewer->removeShape(id);
        viewer->addPolylineFromPolygonMesh(*polymesh,id);
    }
    m_renWnd->Render();
}

void CloudView::updatePolygon(const PolyXYZRGBN &polygon, double r, double g, double b, const string &id)
{
    if(!viewer->contains(id))
        viewer->addPolygon(polygon,r,g,b,id);
    else {
        viewer->removeShape(id);
        viewer->addPolygon(polygon,r,g,b,id);
    }
    m_renWnd->Render();
}

void CloudView::updatePolyLine(const CloudXYZRGBN::Ptr &cloud, double r, double g, double b, const string &id)
{
    if(!viewer->contains(id))
        viewer->addPolygon<PointXYZRGBN>(cloud,r/255,g/255,b/255,id);
    else {
        viewer->removeShape(id);
        viewer->addPolygon<PointXYZRGBN>(cloud,r/255,g/255,b/255,id);
    }
    m_renWnd->Render();
}

void CloudView::updateNormols(const CloudXYZRGBN::Ptr &cloud,int level,float scale,const string &id)
{
    if(!viewer->contains(id))
        viewer->addPointCloudNormals<PointXYZRGBN>(cloud,level,scale,id);
    else {
        viewer->removePointCloud(id);
        viewer->addPointCloudNormals<PointXYZRGBN>(cloud,level,scale,id);
    }
    m_renWnd->Render();
}

void CloudView::updateModel(const Model &model)
{
    viewer->removeShape(model.id);
    ModelCoefs plane;
    switch (model.type)
    {
    case Plane:
        plane.values.resize(4);
        plane.values[0]=model.coefs.values[0];
        plane.values[1]=model.coefs.values[1];
        plane.values[2]=model.coefs.values[2];
        plane.values[3]=model.coefs.values[3];
        viewer->addPlane(plane,model.coefs.values[4],model.coefs.values[5],model.coefs.values[6],model.id);
        break;
    case Sphere:
        viewer->addSphere(model.coefs,model.id);
        break;
    case Line:
        viewer->addLine(model.coefs,model.id);
        break;
    case Cylinder:
        viewer->addCylinder(model.coefs,model.id);
        break;
    case Circle:
        viewer->addCircle(model.coefs,model.id);
        break;
    case Cone:
        viewer->addCone(model.coefs,model.id);
        break;
    case Cube:
        viewer->addCube(model.coefs.values[0],model.coefs.values[3],model.coefs.values[1],
                model.coefs.values[4],model.coefs.values[2],model.coefs.values[5],model.r/255,model.g/255,model.b/255,model.id);
        break;
    case Arrow:
        viewer->addArrow(PointXYZRGBN(model.coefs.values[0],model.coefs.values[1],model.coefs.values[2]),
                PointXYZRGBN(model.coefs.values[3],model.coefs.values[4],model.coefs.values[5]),
                model.r/255,model.g/255,model.b/255,model.display_length,model.id);
        break;
    case Text3D:
        viewer->addText3D(model.text,PointXYZ(model.coefs.values[0],model.coefs.values[1],model.coefs.values[2]),
                model.coefs.values[3],model.r/255,model.g/255,model.b/255,model.id);
        break;
    case Mesh:
        this->updateMesh(model.mesh,model.id);
        break;
    case polygon:
        this->updatePolygon(model.polygon,model.r,model.g,model.b,model.id);
        break;
    case polyline:
        this->updatePolyLine(model.points,model.r,model.g,model.b,model.id);
        break;
    }
    this->setShapeColor(model.id,model.r,model.g,model.b);
    this->setShapeRepersentation(model.id,model.repersentation);
    this->setShapeOpacity(model.id,model.opacity);
    this->setShapeSize(model.id,model.pointSize);
    this->setShapeLineWidth(model.id,model.linewidth);
    this->setShapeFontSize(model.id,model.fontSize);
    this->setShapeShading(model.id,model.shading);
    m_renWnd->Render();
}

PointXYZRGBN CloudView::displayToWorld(const Point2D &pos)
{
    double point[4];
    m_ren->SetDisplayPoint(pos.x,pos.y,0.1);
    m_ren->DisplayToWorld();
    m_ren->GetWorldPoint(point);
    return PointXYZRGBN(point[0],point[1],point[2],0,0,0);
}

Point2D CloudView::worldToDisplay(const PointXYZRGBN &point)
{
    double pos[3];
    m_ren->SetWorldPoint(point.x,point.y,point.z,1);
    m_ren->WorldToDisplay();
    m_ren->GetDisplayPoint(pos);
    return Point2D(pos[0],pos[1]);
}

void CloudView::updateLine(const Point2D &start, const Point2D &end, double r, double g, double b, const string &id)
{
    PointXYZRGBN startPoint=this->displayToWorld(start);
    PointXYZRGBN endPoint=this->displayToWorld(end);
    this->updateLine(startPoint,endPoint,r,g,b,id);
}

void CloudView::updatePolyLine(const std::vector<Point2D> &points, double r, double g, double b, const string &id)
{
    CloudXYZRGBN::Ptr cloud(new CloudXYZRGBN);
    for(int i=0;i<points.size();i++) {
        PointXYZRGBN point=this->displayToWorld(points[i]);
        cloud->push_back(point);
    }
    this->updatePolyLine(cloud,r,g,b,id);
}

void CloudView::updateArrow(const Point2D&start, const Point2D &end, double r, double g, double b, const string &id)
{
    PointXYZRGBN startPoint=this->displayToWorld(start);
    PointXYZRGBN endPoint=this->displayToWorld(end);
    this->updateArrow(startPoint,endPoint,r,g,b,false,id);
}

void CloudView::updateText(const Point2D &start,const string &text, double r, double g, double b,double fontsize,const string &id)
{
    if(!viewer->contains(id))
        viewer->addText(text,start.x,start.y,r,g,b,fontsize,id);
    else {
        viewer->updateText(text,start.x,start.y,r,g,b,fontsize,id);
    }
    m_renWnd->Render();
}

void CloudView::updateCube(const Eigen::Vector3f &translation, const Eigen::Quaternionf &rotation, double width, double height, double depth, const string id)
{
    if(!viewer->contains(id))
        viewer->addCube(translation,rotation,width,height,depth,id);
    else {
        viewer->removeShape(id);
        viewer->addCube(translation,rotation,width,height,depth,id);
    }
    m_renWnd->Render();
}

void CloudView::updateCube(const PointXYZRGBN &min, PointXYZRGBN &max, double r, double g, double b, const string id)
{
    if(!viewer->contains(id))
        viewer->addCube(min.x,max.x,min.y,max.y,min.z,max.z,r/255,g/255,b/255,id);
    else {
        viewer->removeShape(id);
        viewer->addCube(min.x,max.x,min.y,max.y,min.z,max.z,r/255,g/255,b/255,id);
    }
    m_renWnd->Render();
}

//update shape
void CloudView::updateCoord(double scale,const Eigen::Affine3f &pose,const string &id)
{
    viewer->removeCoordinateSystem(id);
    if(scale!=0)
        viewer->addCoordinateSystem(scale,pose,id);
    m_renWnd->Render();
}

void CloudView::updateCoord(double scale, const string &id)
{
    viewer->removeCoordinateSystem(id);
    if(scale!=0)
        viewer->addCoordinateSystem(scale,id);
    m_renWnd->Render();

}

void CloudView::updateCoord(double scale, const Eigen::Vector3f &center, const string &id)
{
    viewer->removeCoordinateSystem(id);
    if(scale!=0)
        viewer->addCoordinateSystem(scale,center[0],center[1],center[2],id);
    m_renWnd->Render();
}

void CloudView::updateLine(const PointXYZRGBN &pt1, const PointXYZRGBN &pt2, double r, double g, double b, const string &id)
{
    if(!viewer->contains(id))
        viewer->addLine(pt1,pt2,r/255,g/255,b/255,id);
    else {
        viewer->removeShape(id);
        viewer->addLine(pt1,pt2,r/255,g/255,b/255,id);
    }
    m_renWnd->Render();
}

void CloudView::updateSphere(const PointXYZRGBN &center, double radius, double r, double g, double b, const string &id)
{
    if(!viewer->contains(id))
        viewer->addSphere(center,radius,r/255,g/255,b/255,id);
    else
        viewer->updateSphere(center,radius,r/255,g/255,b/255,id);
    m_renWnd->Render();
}

void CloudView::updateArrow(const PointXYZRGBN &pt1, const PointXYZRGBN &pt2, double r, double g, double b, bool display_length, const string &id)
{
    if(!viewer->contains(id))
        viewer->addArrow(pt1,pt2,r/255,g/255,b/255,display_length,id);
    else {
        viewer->removeShape(id);
        viewer->addArrow(pt1,pt2,r/255,g/255,b/255,display_length,id);
    }
    m_renWnd->Render();
}

void CloudView::updateText3D(const string &text, const PointXYZRGBN &position, double scale, double r, double g, double b, const string &id)
{
    if(!viewer->contains(id))
        viewer->addText3D(text,position,scale,r/255,g/255,b/255,id);
    else{
        viewer->removeText3D(id);
        viewer->addText3D(text,position,scale,r/255,g/255,b/255,id);
    }
    m_renWnd->Render();
}

void CloudView::updateBoundingBox(const BoundingBox &boundingbox, const string &id)
{
    if(!viewer->contains(id))
        viewer->addCube(boundingbox.translation, boundingbox.rotation, boundingbox.width,
                        boundingbox.height, boundingbox.depth,id);
    else{
        viewer->removeShape(id);
        viewer->addCube(boundingbox.translation, boundingbox.rotation, boundingbox.width,
                        boundingbox.height, boundingbox.depth,id);
    }
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id);
    m_renWnd->Render();
}


//point pick
int CloudView::singlePick(Point2D &pos)
{
    point_picker =vtkSmartPointer<vtkPointPicker>::New ();
    m_renWnd->GetInteractor()->SetPicker (point_picker);
    if (!point_picker) {
        return -1;
    }
    m_renWnd->GetInteractor()->StartPickCallback ();
    vtkRenderer *ren = this->GetInteractor()->FindPokedRenderer (pos.x, pos.y);
    point_picker->Pick (pos.x, pos.y, 0.0, ren);
    return (static_cast<int> (point_picker->GetPointId ()));
}

std::vector<int> CloudView::areaPick(const std::vector<Point2D> &points, const CloudXYZRGBN::Ptr &cloud,bool type)
{
    int size=points.size();
    float  constant[99],multiple[99];
    int   i, j=size-1 ;
    for(i=0; i<size; i++) {
        if(points[j].y==points[i].y) {
            constant[i]=points[i].x;
            multiple[i]=0;
        } else {
            constant[i]=points[i].x-(points[i].y*points[j].x)/(points[j].y-points[i].y)+(points[i].y*points[i].x)/(points[j].y-points[i].y);
            multiple[i]=(points[j].x-points[i].x)/(points[j].y-points[i].y);
        }
        j=i;
    }

    std::vector<int> indices;
    for(size_t i=0;i<cloud->size();i++){
        m_ren->SetWorldPoint(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z,1);
        m_ren->WorldToDisplay();
        double p[3];
        m_ren->GetDisplayPoint(p);

        bool oddNodes=type, current=points[size-1].y>p[1], previous;
        for (int m=0; m<size; m++) {
            previous=current;
            current=points[m].y>p[1];
            if (current!=previous)
                oddNodes^=p[1]*multiple[m]+constant[m]<p[0];
        }
        if(oddNodes)
            indices.push_back(i);
    }
    return indices;
}

//update pose
void CloudView::updateCloudPose(const string &id,const Eigen::Affine3f &affine3f)
{
    viewer->updatePointCloudPose(id,affine3f) ;
    m_renWnd->Render();
}

void CloudView::updateShapePose(const string &shapeid, const Eigen::Affine3f &affine3f)
{
    viewer->updateShapePose(shapeid,affine3f) ;
    m_renWnd->Render();
}

void CloudView::updateCoordPose(const string &id, const Eigen::Affine3f &pose)
{
    viewer->updateCoordinateSystemPose(id,pose);
    m_renWnd->Render();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//remove
void CloudView::removeCloud(const string &id)
{
    viewer->removePointCloud(id);
    m_renWnd->Render();
}

void CloudView::removeMesh(const string &id)
{
    viewer->removePolygonMesh(id);
    m_renWnd->Render();
}

void CloudView::removeShape(const string &shapeid)
{
    viewer->removeShape(shapeid);
    m_renWnd->Render();
}

void CloudView::removeCoord(const string &id)
{
    viewer->removeCoordinateSystem(id);
    m_renWnd->Render();
}

void CloudView::removeText3D(const string &id)
{
    viewer->removeText3D(id);
    m_renWnd->Render();
}

void CloudView::removeCorrespondences(const string &id)
{
    viewer->removeCorrespondences(id);
    m_renWnd->Render();
}

void CloudView::removeAllCoord()
{
    viewer->removeAllCoordinateSystems();
    m_renWnd->Render();
}

void CloudView::removeAllClouds()
{
    viewer->removeAllPointClouds();
    m_renWnd->Render();
}

void CloudView::removeAllShapes()
{
    viewer->removeAllShapes();
    m_renWnd->Render();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//set cloud properties
void CloudView::setCloudColor(const CloudXYZRGBN::Ptr &cloud,const string &id,double r, double g, double b)
{
    pcl::visualization::PointCloudColorHandlerCustom<PointXYZRGBN> color(cloud,r,g,b);
    viewer->updatePointCloud(cloud,color,id);
    m_renWnd->Render();
}

void CloudView::setCloudColor(const CloudXYZRGBN::Ptr &cloud,const string &id,string axis)
{
    pcl::visualization::PointCloudColorHandlerGenericField<PointXYZRGBN> fieldcolor(cloud,axis);
    viewer->updatePointCloud(cloud,fieldcolor,id);
    m_renWnd->Render();
}

void CloudView::resetCloudColor(const CloudXYZRGBN::Ptr &cloud,const string &id)
{
    pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBN>rgb(cloud);
    viewer->updatePointCloud(cloud,rgb,id);
    m_renWnd->Render();
}

void CloudView::setCloudSize(const string &id,float size)
{
    viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, id, 0);
    m_renWnd->Render();
}

void CloudView::setCloudOpacity(const string &id, float value)
{
    viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_OPACITY, value, id, 0);
    m_renWnd->Render();
}

//set background properties
void CloudView::setBackgroundColor(double r, double g, double b)
{
    viewer->setBackgroundColor(r/255.0,g/255.0,b/255.0);
    m_renWnd->Render();
}

void CloudView::resetBackgroundColor()
{
    viewer->setBackgroundColor(150.0 / 255.0, 150.0 / 255.0, 150.0 / 255.0);
    m_renWnd->Render();
}

//set shape properties
void CloudView::setSurfaceForAllShapes()
{
    viewer->setRepresentationToSurfaceForAllActors();
    m_renWnd->Render();
}

void CloudView::setPointsForAllShapes()
{
    viewer->setRepresentationToPointsForAllActors();
    m_renWnd->Render();
}

void CloudView::setWireframeForAllShapes()
{
    viewer->setRepresentationToWireframeForAllActors();
    m_renWnd->Render();
}

void CloudView::setShapeColor(const string &shapeid, double r, double g, double b)
{
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,r/255,g/255,
                                        b/255,shapeid);
    m_renWnd->Render();
}

void CloudView::setShapeSize(const string &shapeid, float size)
{
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,size,shapeid);
    m_renWnd->Render();
}

void CloudView::setShapeOpacity(const string &shapeid, float value)
{
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,value,shapeid);
    m_renWnd->Render();
}

void CloudView::setShapeLineWidth(const string &shapeid, float value)
{
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,value,shapeid);
    m_renWnd->Render();
}

void CloudView::setShapeFontSize(const string &shapeid, float value)
{
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE,value,shapeid);
    m_renWnd->Render();
}

void CloudView::setShapeRepersentation(const string &shapeid, int type)
{
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,type,shapeid);
    m_renWnd->Render();
}

void CloudView::setShapeShading(const string &shapeid, int type)
{
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING,type,shapeid);
    m_renWnd->Render();
}
///////////////////////////////////////////////////////////////////////////////////////////////////
//camera
void CloudView::updateCamera()
{
    viewer->updateCamera();
    m_renWnd->Render();
}

void CloudView::resetCamera()
{
    viewer->resetCamera();
    m_renWnd->Render();
}

void CloudView::resetCameraViewpoint(const std::string &id)
{
    viewer->resetCameraViewpoint(id);
    m_renWnd->Render();
}

Eigen::Affine3f CloudView::getViewerPose()
{
    return viewer->getViewerPose();
}

void CloudView::setViewerPose(const Eigen::Affine3f &viewer_pose)
{
    Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
    Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
    viewer->setCameraPosition(pos_vector[0],pos_vector[1],pos_vector[2],
            look_at_vector[0],look_at_vector[1],look_at_vector[2],
            up_vector[0],up_vector[1],up_vector[2]);
}

void CloudView::initCameraParameters()
{
    viewer->initCameraParameters();
    m_renWnd->Render();
}

void CloudView::saveCameraParameters(const std::string &file)
{
    viewer->saveCameraParameters(file);
    m_renWnd->Render();
}

void CloudView::setCameraParameters(const Eigen::Matrix3f &intrinsics, const Eigen::Matrix4f &extrinsics)
{
    viewer->setCameraParameters(intrinsics,extrinsics);
    m_renWnd->Render();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//display
void CloudView::showInfoText(const string &text, int height, const string &id)
{
    if(!viewer->contains(id))
        viewer->addText(text,10, this->height()-height,11, 1,1,1,id);
    else {
        viewer->updateText(text,10, this->height()-height,11, 1,1,1,id);
    }
    connect(this,&CloudView::sizeChanged,[=](QSize size)
    {
        viewer->updateText(text,10,size.height()-height, 11, 1,1,1,id);
    });
    m_renWnd->Render();
}

void CloudView::showCloudId(const string &id)
{
    if(!showCloudIdEnable) {
        viewer->removeShape("cloudid");
        return;
    }
    if(!viewer->contains("cloudid"))
        viewer->addText(id,this->width()-id.length()*6-10, this->height()-30,11, 1,1,1,"cloudid");
    else
        viewer->updateText(id,this->width()-id.length()*6-10, this->height()-30,11, 1,1,1,"cloudid");
    connect(this,&CloudView::sizeChanged,[=](QSize size)
    {
        viewer->updateText(id, size.width()-id.length()*6-10, size.height()-30, 11, 1,1,1,"cloudid");
    });
    m_renWnd->Render();
}

void CloudView::showAxes()
{
    vtkSmartPointer<vtkAxesActor> axesactor = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetOutlineColor(0.9300, 0.5700, 0.1300);
    axes->SetOrientationMarker(axesactor);
    axes->SetInteractor(this->GetInteractor());
    axes->SetViewport(0.9, 0, 1, 0.15);
    axes->SetEnabled(true);
    axes->InteractiveOn();
    axes->InteractiveOff();
}

void CloudView::setShowCloudId(const bool &enable)
{
    showCloudIdEnable=enable;
    if(!enable)
        viewer->removeShape("cloudid");
    else
        this->showCloudId("CloudTool");
    m_renWnd->Render();
}

void CloudView::setShowFPS(const bool &enable)
{
    viewer->setShowFPS(enable);
    m_renWnd->Render();
}


void CloudView::setshowAxes(const bool &enable)
{
    axes->SetEnabled(enable);
    m_renWnd->Render();
}

void CloudView::setInteractorEnable(const bool &enable)
{
    if(!enable){
        vtkNew<DisableInteractorStyle> style;
        m_renWnd->GetInteractor()->SetInteractorStyle(style);
    } else {
        vtkNew<pcl::visualization::PCLVisualizerInteractorStyle> style;
        m_renWnd->GetInteractor()->SetInteractorStyle(style);
    }
    m_renWnd->Render();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//other
bool CloudView::contains(const string &id)
{
    return viewer->contains(id);
}

void CloudView::saveScreenshot(const string &file)
{
    viewer->saveScreenshot(file);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//viewport
void CloudView::setTopview()
{
    viewer->setCameraPosition(0,0,0, 0, -1, 0, 0, 0, -1);
    viewer->resetCamera();
    m_renWnd->Render();
}

void CloudView::setFrontview()
{
    viewer->setCameraPosition(0,0,0, 0, 0, -1, 0, 1, 0);
    viewer->resetCamera();
    m_renWnd->Render();
}

void CloudView::setLeftSideview()
{
    viewer->setCameraPosition(0,0,0, 1, 0, 0, 0, 1, 0);
    viewer->resetCamera();
    m_renWnd->Render();
}

void CloudView::setBackview()
{
    viewer->setCameraPosition(0,0,0, 0, 0, 1, 0, 1, 0);
    viewer->resetCamera();
    m_renWnd->Render();
}

void CloudView::setRightSideview()
{
    viewer->setCameraPosition(0,0,0, -1, 0, 0, 0, 1, 0);
    viewer->resetCamera();
    m_renWnd->Render();
}

void CloudView::setBottomview()
{
    viewer->setCameraPosition(0,0,0, 0, 1, 0, 0, 0, 1);
    viewer->resetCamera();
    m_renWnd->Render();
}


void CloudView::resizeEvent(QResizeEvent *size)
{
    emit sizeChanged(size->size());
    return QVTKOpenGLNativeWidget::resizeEvent(size);
}

void CloudView::mousePressEvent(QMouseEvent *event)
{
    if(event->button()==Qt::LeftButton){
        emit viewerPose(viewer->getViewerPose());
        emit mouseLeftPressPos(Point2D(m_renWnd->GetInteractor()->GetEventPosition ()[0],
                               m_renWnd->GetInteractor()->GetEventPosition ()[1]));
    } else if(event->button()==Qt::RightButton) {
        emit mouseRightPressPos(Point2D(m_renWnd->GetInteractor()->GetEventPosition ()[0],
                                m_renWnd->GetInteractor()->GetEventPosition ()[1]));
    }
    return QVTKOpenGLNativeWidget::mousePressEvent(event);
}

void CloudView::mouseReleaseEvent(QMouseEvent *event)
{
    if(event->button()==Qt::LeftButton){
        emit mouseLeftReleasePos(Point2D(m_renWnd->GetInteractor()->GetEventPosition ()[0],
                                 m_renWnd->GetInteractor()->GetEventPosition ()[1]));
    } else if(event->button()==Qt::RightButton) {
        emit mouseRightReleasePos(Point2D(m_renWnd->GetInteractor()->GetEventPosition ()[0],
                                  m_renWnd->GetInteractor()->GetEventPosition ()[1]));
    }
    return QVTKOpenGLNativeWidget::mouseReleaseEvent(event);
}

void CloudView::mouseMoveEvent(QMouseEvent *event)
{
    emit mouseMovePos(Point2D(m_renWnd->GetInteractor()->GetEventPosition ()[0],
                      m_renWnd->GetInteractor()->GetEventPosition ()[1]));
    return QVTKOpenGLNativeWidget::mouseReleaseEvent(event);
}

void CloudView::dragEnterEvent(QDragEnterEvent *event)
{
    if(event->mimeData()->hasUrls())
        event->acceptProposedAction();
    else
        event->ignore();
}

void CloudView::dropEvent(QDropEvent *event)
{
    const QMimeData *mimeData=event->mimeData();
    if(mimeData->hasUrls()) {
        QStringList path;
        QList<QUrl> urlList=mimeData->urls();
        for(int i=0;i<urlList.size();i++) {
            QString filepath=urlList.at(i).toLocalFile();
            if(!filepath.isEmpty())
                path.push_back(filepath);
        }
        emit  dropFilePath(path);
    }
}



