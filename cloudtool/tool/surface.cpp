#include "surface.h"
#include "ui_surface.h"

#define SURFACE_TYPE_GreedyProjectionTriangulation  (0)
#define SURFACE_TYPE_GridProjection                 (1)
#define SURFACE_TYPE_Poisson                        (2)
#define SURFACE_TYPE_MarchingCubesRBF               (3)
#define SURFACE_TYPE_MarchingCubesHoppe             (4)
#define SURFACE_TYPE_ConvexHull                     (5)
#define SURFACE_TYPE_ConcaveHull                    (6)

#define SURFACE_PRE_FLAG                            "-surface"
#define SURFACE_ADD_FLAG                            "surface-"

Surface::Surface(QWidget* parent)
    :CustomDock(parent), ui(new Ui::Surface),
    m_thread(this)
{
    ui->setupUi(this);

    qRegisterMetaType<ct::PolygonMesh::Ptr>("PolygonMesh::Ptr &");
    qRegisterMetaType<ct::PolygonMesh::Ptr>("PolygonMesh::Ptr");

    connect(ui->btn_preview, &QPushButton::clicked, this, &Surface::preview);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Surface::reset);

    m_surface = new ct::Surface;
    m_surface->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_surface, &QObject::deleteLater);
    connect(this, &Surface::GreedyProjectionTriangulation, m_surface, &ct::Surface::GreedyProjectionTriangulation);
    connect(this, &Surface::GridProjection, m_surface, &ct::Surface::GridProjection);
    connect(this, &Surface::Poisson, m_surface, &ct::Surface::Poisson);
    connect(this, &Surface::MarchingCubesRBF, m_surface, &ct::Surface::MarchingCubesRBF);
    connect(this, &Surface::MarchingCubesHoppe, m_surface, &ct::Surface::MarchingCubesHoppe);
    connect(this, &Surface::ConvexHull, m_surface, &ct::Surface::ConvexHull);
    connect(this, &Surface::ConcaveHull, m_surface, &ct::Surface::ConcaveHull);

    connect(m_surface, &ct::Surface::surfaceResult, this, &Surface::surfaceResult);
    m_thread.start();

    ui->cbox_surface->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);

    connect(ui->check_polygonline, &QCheckBox::clicked, [=](bool state)
            {
                std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
                for (auto& cloud : selected_clouds)
                {
                    if (m_surface_map.find(cloud->id()) == m_surface_map.end())
                    {
                        printW(QString("The cloud[id:%1] has no surfaced cloud !").arg(cloud->id()));
                        continue;
                    }
                    if (state)
                    {
                        m_cloudview->removePolygonMesh(cloud->id() + SURFACE_PRE_FLAG);
                        m_cloudview->addPolylineFromPolygonMesh(m_surface_map.find(cloud->id())->second, cloud->id() + SURFACE_PRE_FLAG);
                    }
                    else
                    {
                        m_cloudview->removeShape(cloud->id() + SURFACE_PRE_FLAG);
                        m_cloudview->addPolygonMesh(m_surface_map.find(cloud->id())->second, cloud->id() + SURFACE_PRE_FLAG);
                    }
                }
            });
}

Surface::~Surface()
{
    m_thread.quit();
    if (!m_thread.wait(3000))
    {
        m_thread.terminate();
        m_thread.wait();
    }
    delete ui;
}

void Surface::preview()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        m_surface->setInputCloud(cloud);
        switch (ui->cbox_surface->currentIndex())
        {
        case SURFACE_TYPE_GreedyProjectionTriangulation:
            m_cloudview->showInfo("Greedy Projection Triangulation", 1);
            emit GreedyProjectionTriangulation(ui->dspin_mu->value(), ui->spin_nnn->value(), ui->dspin_radius->value(),
                                               ui->spin_minangle->value(), ui->spin_maxangle->value(), ui->spin_maxsurface->value(),
                                               ui->check_consistent->isChecked(), ui->check_consistent_order->isChecked());
            break;
        case SURFACE_TYPE_GridProjection:
            m_cloudview->showInfo("GridProjection", 1);
            emit GridProjection(ui->dspin_leaf_size->value(), ui->spin_padding_size->value(), ui->spin_k->value(),
                                ui->spin_max_binary_search_level->value());
            break;
        case SURFACE_TYPE_Poisson:
            m_cloudview->showInfo("Poisson", 1);
            emit Poisson(ui->spin_depth->value(), ui->spin_min_depth->value(), ui->dspin_point_weight->value(),
                         ui->dspin_scale->value(), ui->spin_solver_divide->value(), ui->spin_iso_divide->value(),
                         ui->dspin_samples_per_node->value(), ui->check_confidence->isChecked(),
                         ui->check_out_polygons->isChecked(), ui->check_manifold->isChecked());
            break;
        case SURFACE_TYPE_MarchingCubesRBF:
            m_cloudview->showInfo("MarchingCubesRBF", 1);
            emit MarchingCubesRBF(ui->dspin_iso_level->value(), ui->spin_res_x->value(), ui->spin_res_y->value(),
                                  ui->spin_res_z->value(), ui->dspin_percentage->value(), ui->dspin_epslion->value());
            break;
        case SURFACE_TYPE_MarchingCubesHoppe:
            m_cloudview->showInfo("MarchingCubesHoppe", 1);
            emit MarchingCubesHoppe(ui->dspin_iso_level_2->value(), ui->spin_res_x_2->value(), ui->spin_res_y_2->value(),
                                    ui->spin_res_z_2->value(), ui->dspin_percentage_2->value(), ui->dspin_dist_ignore->value());
            break;
        case SURFACE_TYPE_ConvexHull:
            m_cloudview->showInfo("ConvexHull", 1);
            emit ConvexHull(ui->check_value->isChecked(), ui->spin_dimensio->value());
            break;
        case SURFACE_TYPE_ConcaveHull:
            m_cloudview->showInfo("ConcaveHull", 1);
            emit ConcaveHull(ui->dspin_alpha->value(), ui->check_value_2->isChecked(), ui->spin_dimensio_2->value());
            break;
        }
        m_cloudtree->showProgressBar();
    }
}

void Surface::reset()
{
    for (auto& cloud : m_surface_map)
    {
        m_cloudview->removePolygonMesh(cloud.first + SURFACE_PRE_FLAG);
        m_cloudview->removeShape(cloud.first + SURFACE_PRE_FLAG);
    }
    m_surface_map.clear();
    m_cloudview->clearInfo();
}

void Surface::surfaceResult(const QString& id, const ct::PolygonMesh::Ptr& surface, float time)
{
    printI(QString("surface cloud[id:%1] done, take time %2 ms.").arg(id).arg(time));
    if (ui->check_polygonline->isChecked())
        m_cloudview->addPolylineFromPolygonMesh(surface, id + SURFACE_PRE_FLAG);
    else
        m_cloudview->addPolygonMesh(surface, id + SURFACE_PRE_FLAG);
    m_surface_map[id] = surface;
    m_cloudtree->closeProgressBar();
}