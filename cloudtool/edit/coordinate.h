/**
 * @file coordinate.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#ifndef CT_EDIT_COORDINATE_H
#define CT_EDIT_COORDINATE_H

#include "base/customdialog.h"

namespace Ui
{
    class Coordinate;
}

class Coordinate : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit Coordinate(QWidget* parent = nullptr);
    ~Coordinate();

    virtual void init();
    void add();
    virtual void reset();
    virtual void deinit() { m_cloudview->removeAllCoordinateSystems();}
    void addCoord();
    void closeCoord();

private:
    Ui::Coordinate* ui;
    ct::Coord m_origin_coord; // origin_coord
    std::unordered_map<QString, ct::Coord> m_coord_map; // cloud coord
};

#endif  // COORDS_H
