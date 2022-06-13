#include "cloudview.h"
CloudView::CloudView(QWidget *parent) :QVTKOpenGLNativeWidget(parent),
    showId_enable(true),render(vtkSmartPointer<vtkRenderer>::New()),
    render_window(vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New()),
    axes(vtkSmartPointer<vtkOrientationMarkerWidget>::New())
{

}

void CloudView::init()
{
    render_window->AddRenderer(render);
    viewer.reset(new pcl::visualization::PCLVisualizer(render,render_window,"viewer", false));
    this->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(this->GetInteractor(),this->GetRenderWindow());
    viewer->setBackgroundColor(150.0 / 255.0, 150.0 / 255.0, 150.0 / 255.0);
    vtkSmartPointer<vtkAxesActor> axesactor = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetOutlineColor(0.9300, 0.5700, 0.1300);
    axes->SetOrientationMarker(axesactor);
    axes->SetInteractor(this->GetInteractor());
    axes->SetViewport(0.9, 0, 1, 0.15);
    axes->SetEnabled(true);
    axes->InteractiveOn();
    axes->InteractiveOff();
    render_window->Render();
}

void CloudView::addCloud(const Cloud::Ptr &cloud,const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addPointCloud<PointXYZRGBN>(cloud,id);
    else {
        pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBN>rgb(cloud);
        viewer->updatePointCloud<PointXYZRGBN>(cloud,rgb,id);
    }
    render_window->Render();
}

void CloudView::addCoord(double scale,const std::string &id)
{
    viewer->removeCoordinateSystem(id);
    if(scale!=0)
        viewer->addCoordinateSystem(scale,id);
    render_window->Render();
}

void CloudView::addCoord(double scale,const Eigen::Affine3f &affine,const std::string &id)
{
    viewer->removeCoordinateSystem(id);
    if(scale!=0)
        viewer->addCoordinateSystem(scale,affine,id);
    render_window->Render();
}

void CloudView::addCoord(double scale, float x, float y, float z, const std::string &id)
{
    viewer->removeCoordinateSystem(id);
    if(scale!=0)
        viewer->addCoordinateSystem(scale,x,y,z,id);
    render_window->Render();
}

void CloudView::addText (const std::string &text,int xpos, int ypos,const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addText(text,xpos,ypos,id);
    else
        viewer->updateText(text,xpos,ypos,id);
    render_window->Render();
}

void CloudView::addText (const std::string &text, int xpos, int ypos, double r, double g, double b,
                         const std::string &id )
{
    if(!viewer->contains(id))
        viewer->addText(text,xpos,ypos,r/255,g/255,b/255,id);
    else
        viewer->updateText(text,xpos,ypos,r/255,g/255,b/255,id);
    render_window->Render();
}

void CloudView::addText (const std::string &text, int xpos, int ypos, int fontsize, double r, double g, double b,
                         const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addText(text,xpos,ypos,fontsize,r/255,g/255,b/255,id);
    else
        viewer->updateText(text,xpos,ypos,fontsize,r/255,g/255,b/255,id);
    render_window->Render();
}

void CloudView::addText3D (const std::string &text,const PointXYZRGBN &position,double textScale,double r, double g, double b,
                           const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addText3D(text,position,textScale,r/255,g/255,b/255,id);
    else {
        viewer->removeShape(id);
        viewer->addText3D(text,position,textScale,r/255,g/255,b/255,id);
    }
    render_window->Render();
}

void CloudView::addCloudNormals (const Cloud::Ptr &cloud,int level, float scale,const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addPointCloudNormals<PointXYZRGBN>(cloud,level,scale,id);
    else {
        viewer->removePointCloud(id);
        viewer->addPointCloudNormals<PointXYZRGBN>(cloud,level,scale,id);
    }
    render_window->Render();
}

void CloudView::addCloudNormals (const Cloud::Ptr &cloud,const CloudNormal::Ptr &normals,int level, float scale,
                                 const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addPointCloudNormals<PointXYZRGBN,Normal>(cloud,normals,level,scale,id);
    else {
        viewer->removePointCloud(id);
        viewer->addPointCloudNormals<PointXYZRGBN,Normal>(cloud,normals,level,scale,id);
    }
    render_window->Render();
}

void CloudView::addCloudPrincipalCurvatures (const Cloud::Ptr &cloud,const PrincipalCurvatures::Ptr &pcs,
                                             int level, float scale,const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addPointCloudPrincipalCurvatures<PointXYZRGBN>(cloud,pcs,level,scale,id);
    else {
        viewer->removePointCloud(id);
        viewer->addPointCloudPrincipalCurvatures<PointXYZRGBN>(cloud,pcs,level,scale,id);
    }
    render_window->Render();
}

void CloudView::addCloudPrincipalCurvatures (const Cloud::Ptr &cloud,const CloudNormal::Ptr &normals,
                                             const PrincipalCurvatures::Ptr &pcs,int level, float scale,const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addPointCloudPrincipalCurvatures<PointXYZRGBN,Normal>(cloud,normals,pcs,level,scale,id);
    else {
        viewer->removePointCloud(id);
        viewer->addPointCloudPrincipalCurvatures<PointXYZRGBN>(cloud,pcs,level,scale,id);
    }
    render_window->Render();
}

void CloudView::addPolygonMesh (const PolygonMesh::Ptr &polymesh,const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addPolygonMesh(*polymesh,id);
    else{
        viewer->removePolygonMesh(id);
        viewer->addPolygonMesh(*polymesh,id);
    }
    render_window->Render();
}

void CloudView::addPolylineFromPolygonMesh (const PolygonMesh::Ptr &polymesh,const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addPolylineFromPolygonMesh(*polymesh,id);
    else {
        viewer->removeShape(id);
        viewer->addPolylineFromPolygonMesh(*polymesh,id);
    }
    render_window->Render();
}

void CloudView::addCorrespondences (const Cloud::Ptr &source_points,const Cloud::Ptr &target_points,
                                    const CorrespondencesPtr &correspondences,const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addCorrespondences<PointXYZRGBN>(source_points,target_points,*correspondences,id);
    else
        viewer->updateCorrespondences<PointXYZRGBN>(source_points,target_points,*correspondences,id);
    render_window->Render();
}

void CloudView::addPolygon (const Cloud::Ptr &cloud,const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addPolygon<PointXYZRGBN>(cloud,id);
    else {
        viewer->removeShape(id);
        viewer->addPolygon<PointXYZRGBN>(cloud,id);
    }
    render_window->Render();
}

void CloudView::addPolygon (const Cloud::Ptr &cloud,double r, double g, double b,const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addPolygon<PointXYZRGBN>(cloud,r/255,g/255,b/255,id);
    else {
        viewer->removeShape(id);
        viewer->addPolygon<PointXYZRGBN>(cloud,r/255,g/255,b/255,id);
    }
    render_window->Render();
}

void CloudView::addPolygon (const PolyXYZRGBN::Ptr &polygon,double r, double g, double b,const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addPolygon<PointXYZRGBN>(*polygon,r/255,g/255,b/255,id);
    else {
        viewer->removeShape(id);
        viewer->addPolygon<PointXYZRGBN>(*polygon,r/255,g/255,b/255,id);
    }
    render_window->Render();
}

void CloudView::addLine (const PointXYZRGBN &pt1,const PointXYZRGBN &pt2,const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addLine(pt1,pt2,id);
    else {
        viewer->removeShape(id);
        viewer->addLine(pt1,pt2,id);
    }
    render_window->Render();
}

void CloudView::addLine (const PointXYZRGBN &pt1,const PointXYZRGBN &pt2,double r, double g, double b,const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addLine(pt1,pt2,r/255,g/255,b/255,id);
    else {
        viewer->removeShape(id);
        viewer->addLine(pt1,pt2,r/255,g/255,b/255,id);
    }
    render_window->Render();
}

void CloudView::addArrow (const PointXYZRGBN &pt1,const PointXYZRGBN &pt2,double r, double g, double b,
                          bool display_length,const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addArrow(pt1,pt2,r/255,g/255,b/255,display_length,id);
    else {
        viewer->removeShape(id);
        viewer->addArrow(pt1,pt2,r/255,g/255,b/255,display_length,id);
    }
    render_window->Render();
}

void CloudView::addSphere(const PointXYZRGBN &center, double radius, const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addSphere(center,radius,id);
    else
        viewer->updateSphere(center,radius,0,0,0,id);
    render_window->Render();
}

void CloudView::addSphere(const PointXYZRGBN &center, double radius, double r, double g, double b,const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addSphere(center,radius,r/255,g/255,b/255,id);
    else
        viewer->updateSphere(center,radius,r/255,g/255,b/255,id);
    render_window->Render();
}

void CloudView::addCylinder(const ModelCoefficients::Ptr &coefficients, const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addCylinder(*coefficients,id);
    else {
        viewer->removeShape(id);
        viewer->addCylinder(*coefficients,id);
    }
    render_window->Render();
}

void CloudView::addSphere(const ModelCoefficients::Ptr &coefficients, const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addSphere(*coefficients,id);
    else {
        viewer->removeShape(id);
        viewer->addSphere(*coefficients,id);
    }
    render_window->Render();
}

void CloudView::addLine(const ModelCoefficients::Ptr &coefficients, const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addLine(*coefficients,id);
    else {
        viewer->removeShape(id);
        viewer->addLine(*coefficients,id);
    }
    render_window->Render();
}

void CloudView::addPlane(const ModelCoefficients::Ptr &coefficients, const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addPlane(*coefficients,id);
    else {
        viewer->removeShape(id);
        viewer->addPlane(*coefficients,id);
    }
    render_window->Render();
}

void CloudView::addPlane(const ModelCoefficients::Ptr &coefficients, double x, double y, double z, const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addPlane(*coefficients,x,y,z,id);
    else {
        viewer->removeShape(id);
        viewer->addPlane(*coefficients,x,y,z,id);
    }
    render_window->Render();
}

void CloudView::addCircle(const ModelCoefficients::Ptr &coefficients, const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addCircle(*coefficients,id);
    else {
        viewer->removeShape(id);
        viewer->addCircle(*coefficients,id);
    }
    render_window->Render();
}

void CloudView::addCone(const ModelCoefficients::Ptr &coefficients, const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addCone(*coefficients,id);
    else {
        viewer->removeShape(id);
        viewer->addCone(*coefficients,id);
    }
    render_window->Render();
}

void CloudView::addCube(const ModelCoefficients::Ptr &coefficients, const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addCube(*coefficients,id);
    else {
        viewer->removeShape(id);
        viewer->addCube(*coefficients,id);
    }
    render_window->Render();
}

void CloudView::addCube(const Eigen::Vector3f &translation, const Eigen::Quaternionf &rotation, double width, double height, double depth, const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addCube(translation,rotation,width,height,depth,id);
    else {
        viewer->removeShape(id);
        viewer->addCube(translation,rotation,width,height,depth,id);
    }
    render_window->Render();
}

void CloudView::addCube(const PointXYZRGBN &min, PointXYZRGBN &max, double r, double g, double b, const std::string id)
{
    if(!viewer->contains(id))
        viewer->addCube(min.x,max.x,min.y,max.y,min.z,max.z,r/255,g/255,b/255,id);
    else {
        viewer->removeShape(id);
        viewer->addCube(min.x,max.x,min.y,max.y,min.z,max.z,r/255,g/255,b/255,id);
    }
    render_window->Render();
}

void CloudView::addCube(const Box &box,const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addCube(box.translation,box.rotation,box.width,box.height,box.depth,id);
    else {
        viewer->removeShape(id);
        viewer->addCube(box.translation,box.rotation,box.width,box.height,box.depth,id);
    }
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id);

    render_window->Render();
}

PointXYZRGBN CloudView::displayToWorld(const Point2D &pos)
{
    double point[4];
    render->SetDisplayPoint(pos.x,pos.y,0.1);
    render->DisplayToWorld();
    render->GetWorldPoint(point);
    return PointXYZRGBN(point[0],point[1],point[2],0,0,0);
}

Point2D CloudView::worldToDisplay(const PointXYZRGBN &point)
{
    double pos[3];
    render->SetWorldPoint(point.x,point.y,point.z,1);
    render->WorldToDisplay();
    render->GetDisplayPoint(pos);
    return Point2D(pos[0],pos[1]);
}

void CloudView::addLine(const Point2D &start, const Point2D &end, double r, double g, double b, const std::string &id)
{
    PointXYZRGBN startPoint=this->displayToWorld(start);
    PointXYZRGBN endPoint=this->displayToWorld(end);
    this->addLine(startPoint,endPoint,r,g,b,id);
}

void CloudView::addPolyLine(const std::vector<Point2D> &points, double r, double g, double b, const std::string &id)
{
    Cloud::Ptr cloud(new Cloud);
    for(auto &i:points) {
        PointXYZRGBN point=this->displayToWorld(i);
        cloud->push_back(point);
    }
    this->addPolygon(cloud,r,g,b,id);
}

void CloudView::addArrow(const Point2D&start, const Point2D &end, double r, double g, double b, const std::string &id)
{
    PointXYZRGBN startPoint=this->displayToWorld(start);
    PointXYZRGBN endPoint=this->displayToWorld(end);
    this->addArrow(startPoint,endPoint,r,g,b,false,id);
}

void CloudView::addText(const Point2D &start,const std::string &text, double r, double g, double b,double fontsize,const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addText(text,start.x,start.y,r,g,b,fontsize,id);
    else {
        viewer->updateText(text,start.x,start.y,r,g,b,fontsize,id);
    }
    render_window->Render();
}

int CloudView::singlePick(const Point2D &pos)
{
    vtkSmartPointer<vtkPointPicker> m_point_picker=vtkSmartPointer<vtkPointPicker>::New ();
    render_window->GetInteractor()->SetPicker (m_point_picker);
    if (!m_point_picker)
        return -1;
    render_window->GetInteractor()->StartPickCallback ();
    vtkRenderer *ren = this->GetInteractor()->FindPokedRenderer (pos.x, pos.y);
    m_point_picker->Pick (pos.x, pos.y, 0.0, ren);
    return (static_cast<int> (m_point_picker->GetPointId ()));
}

std::vector<int> CloudView::areaPick(const std::vector<Point2D> &points, const Cloud::Ptr &cloud, bool type)
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
        render->SetWorldPoint(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z,1);
        render->WorldToDisplay();
        double p[3];
        render->GetDisplayPoint(p);

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

void CloudView::updateCloud(const Cloud::Ptr &cloud,const std::string &id)
{
    pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBN>rgb(cloud);
    viewer->updatePointCloud<PointXYZRGBN>(cloud,rgb,id);
    render_window->Render();
}

void CloudView::setCloudSelected(const bool selected, const std::string &id)
{
    viewer->setPointCloudSelected(selected,id);
    render_window->Render();
}

void CloudView::setCloudColor(const Cloud::Ptr &cloud,const std::string &id,double r, double g, double b)
{
    pcl::visualization::PointCloudColorHandlerCustom<PointXYZRGBN> color(cloud,r,g,b);
    viewer->updatePointCloud(cloud,color,id);
    render_window->Render();
}

void CloudView::setCloudColor(const Cloud::Ptr &cloud,const std::string &id,const std::string& axis)
{
    pcl::visualization::PointCloudColorHandlerGenericField<PointXYZRGBN> fieldcolor(cloud,axis);
    viewer->updatePointCloud(cloud,fieldcolor,id);
    render_window->Render();
}

void CloudView::resetCloudColor(const Cloud::Ptr &cloud,const std::string &id)
{
    pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBN>rgb(cloud);
    viewer->updatePointCloud(cloud,rgb,id);
    render_window->Render();
}

void CloudView::setCloudSize(const std::string &id,float size)
{
    viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, id, 0);
    render_window->Render();
}

void CloudView::setCloudOpacity(const std::string &id, float value)
{
    viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_OPACITY, value, id, 0);
    render_window->Render();
}

void CloudView::setBackgroundColor(double r, double g, double b)
{
    viewer->setBackgroundColor(r/255.0,g/255.0,b/255.0);
    render_window->Render();
}

void CloudView::resetBackgroundColor()
{
    viewer->setBackgroundColor(150.0 / 255.0, 150.0 / 255.0, 150.0 / 255.0);
    render_window->Render();
}

void CloudView::setShapeColor(const std::string &shapeid, double r, double g, double b)
{
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,r/255,g/255,
                                        b/255,shapeid);
    render_window->Render();
}

void CloudView::setShapeSize(const std::string &shapeid, float size)
{
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,size,shapeid);
    render_window->Render();
}

void CloudView::setShapeOpacity(const std::string &shapeid, float value)
{
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,value,shapeid);
    render_window->Render();
}

void CloudView::setShapeLineWidth(const std::string &shapeid, float value)
{
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,value,shapeid);
    render_window->Render();
}

void CloudView::setShapeFontSize(const std::string &shapeid, float value)
{
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE,value,shapeid);
    render_window->Render();
}

void CloudView::setShapeRepersentation(const std::string &shapeid, int type)
{
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,type,shapeid);
    render_window->Render();
}

void CloudView::setShapeShading(const std::string &shapeid, int type)
{
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING,type,shapeid);
    render_window->Render();
}

void CloudView::setSurfaceForAllShapes()
{
    viewer->setRepresentationToSurfaceForAllActors();
    render_window->Render();
}

void CloudView::setPointsForAllShapes()
{
    viewer->setRepresentationToPointsForAllActors();
    render_window->Render();
}

void CloudView::setWireframeForAllShapes()
{
    viewer->setRepresentationToWireframeForAllActors();
    render_window->Render();
}


void CloudView::setViewerPose(const Eigen::Affine3f &pose)
{
    Eigen::Vector3f pos_vector = pose * Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f look_at_vector = pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
    Eigen::Vector3f up_vector = pose.rotation() * Eigen::Vector3f(0, -1, 0);
    viewer->setCameraPosition(pos_vector[0],pos_vector[1],pos_vector[2],
            look_at_vector[0],look_at_vector[1],look_at_vector[2],
            up_vector[0],up_vector[1],up_vector[2]);
    render_window->Render();
}

void CloudView::showInfo(const std::string &text, int height, const std::string &id)
{
    if(!viewer->contains(id))
        viewer->addText(text,10, this->height()-height,11, 1,1,1,id);
    else
        viewer->updateText(text,10, this->height()-height,11, 1,1,1,id);
    connect(this,&CloudView::sizeChanged,[=](QSize size)
    {
        viewer->updateText(text,10,size.height()-height, 11, 1,1,1,id);
    });
    render_window->Render();
}

void CloudView::showId(const std::string &id)
{
    if(!showId_enable)return;
    if(!viewer->contains("cloudid"))
        viewer->addText(id,this->width()-id.length()*6-10, this->height()-30,11, 1,1,1,"cloudid");
    else
        viewer->updateText(id,this->width()-id.length()*6-10, this->height()-30,11, 1,1,1,"cloudid");
    connect(this,&CloudView::sizeChanged,[=](QSize size)
    {
        viewer->updateText(id, size.width()-id.length()*6-10, size.height()-30, 11, 1,1,1,"cloudid");
    });
    render_window->Render();
}

void CloudView::setShowId(const bool &enable)
{
    showId_enable=enable;
    if(!enable)
        viewer->removeShape("cloudid");
    else
        this->showId("CloudTool");
    render_window->Render();
}

void CloudView::setInteractorEnable(const bool &enable)
{
    if(!enable){
        vtkNew<DisableInteractorStyle> style;
        render_window->GetInteractor()->SetInteractorStyle(style);
    } else {
        vtkNew<pcl::visualization::PCLVisualizerInteractorStyle> style;
        render_window->GetInteractor()->SetInteractorStyle(style);
    }
    render_window->Render();
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

void CloudView::resizeEvent(QResizeEvent *size)
{
    emit sizeChanged(size->size());
    return QVTKOpenGLNativeWidget::resizeEvent(size);
}

void CloudView::mousePressEvent(QMouseEvent *event)
{
    if(event->button()==Qt::LeftButton){
        emit mouseLeftPressed(Point2D(render_window->GetInteractor()->GetEventPosition ()[0],
                              render_window->GetInteractor()->GetEventPosition ()[1]));
    } else if(event->button()==Qt::RightButton) {
        emit mouseRightPressed(Point2D(render_window->GetInteractor()->GetEventPosition ()[0],
                               render_window->GetInteractor()->GetEventPosition ()[1]));
    }
    return QVTKOpenGLNativeWidget::mousePressEvent(event);
}

void CloudView::mouseReleaseEvent(QMouseEvent *event)
{
    if(event->button()==Qt::LeftButton){
        emit mouseLeftReleased(Point2D(render_window->GetInteractor()->GetEventPosition ()[0],
                               render_window->GetInteractor()->GetEventPosition ()[1]));
    } else if(event->button()==Qt::RightButton) {
        emit mouseRightReleased(Point2D(render_window->GetInteractor()->GetEventPosition ()[0],
                                render_window->GetInteractor()->GetEventPosition ()[1]));
    }
    return QVTKOpenGLNativeWidget::mouseReleaseEvent(event);
}

void CloudView::mouseMoveEvent(QMouseEvent *event)
{
    emit mouseMoved(Point2D(render_window->GetInteractor()->GetEventPosition ()[0],
                    render_window->GetInteractor()->GetEventPosition ()[1]));
    return QVTKOpenGLNativeWidget::mouseReleaseEvent(event);
}

