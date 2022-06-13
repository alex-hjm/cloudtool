#ifndef QENHANCEDGRAPHICSVIEW_H
#define QENHANCEDGRAPHICSVIEW_H

#include <QWidget>
#include <QGraphicsView>
#include <QMimeData>
#include <QUrl>
#include <QDropEvent>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QtMath>
#include <QContextMenuEvent>
#include <QMenu>
#include <QGraphicsItem>
#include <QDebug>
#include <QGraphicsEffect>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

class GraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit GraphicsView(QWidget *parent = nullptr);

    void addImage(const cv::Mat &imagein);
    void removeImage();

    QImage Mat2QImage(const cv::Mat &imagein);
    cv::Mat QImage2Mat(const QImage &image, bool inCloneImageData);
protected:
    void wheelEvent(QWheelEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void dragEnterEvent(QDragEnterEvent *event);
    void dragMoveEvent(QDragMoveEvent  *event);
    void dropEvent(QDropEvent *event);


signals:
    void dropFilePath(QStringList);

public slots:

private slots:
    void clearAll(bool);
    void clearSelected(bool);
    void noEffect(bool);
    void blurEffect(bool);
    void dropShadowEffect(bool);
    void colorizeEffect(bool);
    void customEffect(bool);

private:
    QPointF sceneMousePos;
    QGraphicsScene m_scene;
};

#endif // QENHANCEDGRAPHICSVIEW_H
