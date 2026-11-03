/*
 *    Copyright (C) 2026 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <tuple>
#include <QImage>
#include <QPainter>
#include <QPen>
#include <QPixmap>
#include <Eigen/Dense>
#include <Eigen/Core>       // Para Eigen::Vector3d
#include <Eigen/Geometry> 



/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    /**
     * \brief Constructor for SpecificWorker.
     * \param configLoader Configuration loader for the component.
     * \param tprx Tuple of proxies required for the component.
     * \param startup_check Indicates whether to perform startup checks.
     */
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);

	/**
     * \brief Destructor for SpecificWorker.
     */
	~SpecificWorker();


public slots:

	void new_target_slot(QPointF);

	/**
	 * \brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * \brief Main compute loop of the worker.
	 */
	void compute();

	/**
	 * \brief Handles the emergency state loop.
	 */
	void emergency();

	/**
	 * \brief Restores the component from an emergency state.
	 */
	void restore();

    /**
     * \brief Performs startup checks for the component.
     * \return An integer representing the result of the checks.
     */
	int startup_check();

private:
	QRectF dimensions;
	AbstractGraphicViewer *viewer;
	const int ROBOT_LENGTH = 400;
	QGraphicsItem *robot_polygon;

	enum class STATE {
		IDLE,
		FORWARD,
		TURN,
		FOLLOW_WALL,
		SPIRAL
	};
	STATE state = STATE::IDLE;

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

	//Choca choca Practica 1
	void draw_lidar(const auto &points, QGraphicsScene* scene);
	RoboCompLidar3D::TPoints cono(const RoboCompLidar3D::TPoints &points, float minAngle, float maxAngle, float distance);
	std::tuple<STATE, float, float> navigation(const RoboCompLidar3D::TPoints &points);
	void move_robot(float adv, float rot);
	//Localiser Practica 2 
	void draw_image(const RoboCompCamera360RGB::TImage &raw, const QRect &roi, const std::vector<QPointF> &projected_points,  QLabel *label);
	void draw_person_in_lidar(float person_angle, float distance, QGraphicsScene* scene);
	std::optional<std::tuple<std::vector<QPointF>, float>> project_lidar_to_camera(
                                           const RoboCompLidar3D::TPoints& points,
                                           const Eigen::Isometry3d& cam_transform,
                                           double personAngle,
                                           const RoboCompCamera360RGB::TImage &image,
                                           const QRect &roi,
                                           double azimuth_sign = 1.0,
                                           double azimuth_offset = 0.0);
	

	
signals:
	//void customSignal();
};

#endif
