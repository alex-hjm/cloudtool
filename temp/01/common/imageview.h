#ifndef IMAGEVIEW_H
#define IMAGEVIEW_H

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
#include <QDebug>
#include <QGraphicsItem>
#include <QGraphicsEffect>
#include "image.h"

class ImageView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit ImageView(QWidget *parent = nullptr);

    void addImage(const Image::Ptr&image,bool fit);
    void removeImage();
    void resetView();

    QImage Mat2QImage(const cv::Mat &image);
    cv::Mat QImage2Mat(const QImage &image, bool inCloneImageData);
protected:
    void wheelEvent(QWheelEvent *event);
    void mousePressEvent(QMouseEvent *event);

    void dragEnterEvent(QDragEnterEvent *event);
    void dragMoveEvent(QDragMoveEvent  *event);
    void dropEvent(QDropEvent *event);

signals:
    void dropFilePath(const QStringList &filepath);

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
    bool m_isMoving;
};

#endif // IMAGEVIEW_H
