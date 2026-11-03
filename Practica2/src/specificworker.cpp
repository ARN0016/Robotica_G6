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
#include "specificworker.h"

const float SAFETY_THRESHOLD = 635.0f;

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif
		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;
	GenericWorker::initialize();


	this->dimensions = QRectF(-6000, -3000, 12000, 6000);
	viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
	this->resize(900,450);
	viewer->show();
	const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
	robot_polygon = std::get<0>(rob);

	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

    //initializeCODE
    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 
}



void SpecificWorker::compute()
{
    fps.print("Compute worker", 3000);
	RoboCompLidar3D::TData lidar_data;


	//computeCODE
	try
	// ppoints.front().r
	{
		lidar_data = lidar3d_proxy->getLidarData ("lidar", 0.f, 2*M_PI, 3);
		draw_lidar(lidar_data.points, &viewer->scene);

	}
	catch(const Ice::Exception &e)
	{
	  std::cout << "Error reading from Lidar" << e << std::endl;
	}

	

	RoboCompVisualElements::TObjects objects;
	QRect roi;
	RoboCompVisualElements::TObject person;

	try{
		objects = visualelements_proxy->getVisualObjects(RoboCompVisualElements::TObjects{});
	} catch(const Ice::Exception &e){
 		std::cout << "Exception in getVisualObjects: " << e.what() << std::endl;
		return;
	}
	if (objects.objects.empty()) {
		qWarning() << "no person detected"; 
		return;
	}

	person = objects.objects.front();
	roi = QRect(person.left, person.top, person.right - person.left, person.bot - person.top);

	//Get Image
	RoboCompCamera360RGB::TImage image;

	try{
		image = camera360rgb_proxy->getROI(0, 0, 0, 0, 640, 480);
	}
	catch(const Ice::Exception &e){
 		std::cout << "Ice Exception in getROI: " << e.what() << std::endl; 
		return;
	}
	if (image.width == 0) {
		qWarning() << "Empty image "; 
		return;
	}
	double person_offset = (image.width/2.0) + (person.right - person.left)/2.0 - person.left;
	double person_angle = -(2.0*M_PI / image.width) * person_offset + 0.1;

	Eigen::Isometry3d cam_transform = Eigen::Isometry3d::Identity();
	cam_transform.pretranslate(-Eigen::Vector3d{0.f, -170.f,1275.f});
	std::tuple<std::vector<QPointF>, float> res;
	if(const auto res_ = project_lidar_to_camera(lidar_data.points, cam_transform, person_angle, image, roi); not res_.has_value()){
		qWarning() << "No lidar projection"; 
		return;
	}
	else res = res_.value();
	const auto&[projected_points, depth_from_proj] = res ; //No esta creado nada

	const double current_time = QDateTime::currentDateTime().toMSecsSinceEpoch() / 1000.0;
	//const auto &[corrected_angle, corrected_distance] = target_tracked.update(person_angle, depth_from_proj, current_time); 
	
	//call controler
	//const auto &[adv, rot] = controller.update(person_angle, depth_from_proj);
	//const auto &[adv,rot] = controller.update(corrected_angle, corrected_distance);

	//move robot
	try { omnirobot_proxy->setSpeedBase (0, brake(person_angle) * (depth_from_proj -1000), -person_angle * 1.5); }
	catch (const Ice::Exception &e) 
	{
		std::cout << "Exception in setBaseSpeed: " << e.what() << std::endl;
		return;
	}

	draw_image(image, roi, projected_points, label);
	//draw_image(image, roi, {}, label);

	//TODO Implementar este metodo
	//draw_person_in_lidar(corrected_angle, corrected_distance, &viewer->scene);
	draw_person_in_lidar(person_angle, depth_from_proj, &viewer->scene);
	draw_lidar(lidar_data.points, &viewer->scene);

	fps.print("Compute Worker", 3000);
	
	 //choca choca
	// const auto &[STATE, adv, rot] = navigation(data.points);
	
	// std::cout << "navigation: " << adv <<" " << rot<< std::endl;
	// move_robot(adv, rot);
}

float SpecificWorker::brake(float angle)
{
	const double omega = - std::pow(0.5,2) /std::log(0.5);
	double y = std::exp(-(angle * angle) / omega);
	return y;
}

//------------------------------------------------------------------------------------

void SpecificWorker::draw_image(const RoboCompCamera360RGB::TImage &raw, const QRect &roi, const std::vector<QPointF> &projected_points,  QLabel *label)
{
   QImage img(raw.image.data(),
              raw.width,
              raw.height,
              raw.width * 3,
              QImage::Format_RGB888);
			  
    QSize targetSize = label->size();

	QImage resized = img.scaled(
       targetSize,
       Qt::IgnoreAspectRatio,
       Qt::SmoothTransformation
   );


   double sx = double(resized.width()) / raw.width;
   double sy = double(resized.height()) / raw.height;


   QPainter painter(&resized);


   QPen pen(Qt::red, 2);
   pen.setCosmetic(true);
   painter.setPen(pen);


   painter.scale(sx, sy);


   painter.drawRect(roi);


   // Draw projected points
   pen.setColor(Qt::blue);
   painter.setPen(pen);
   for (const auto& pt : projected_points)
       painter.drawPoint(pt);


   // Draw a vertical line at mid-width with thickness proportional to image height
   const double image_mid_x = raw.width / 2.0;
   const int vertical_guide_thickness = std::clamp<int>(static_cast<int>(raw.height / 60.0), 2, 12);
   pen.setColor(Qt::green);
   pen.setWidth(vertical_guide_thickness);
   pen.setCosmetic(false);
   painter.setPen(pen);
   painter.drawLine(QPointF(image_mid_x, 0.0), QPointF(image_mid_x, raw.height));


   // Draw a horizontal line from the closest ROI side to the vertical guide
   const double roi_mid_y = roi.top() + roi.height() / 2.0;
   const double roi_left_dist = std::abs(roi.left() - image_mid_x);
   const double roi_right_dist = std::abs(roi.right() - image_mid_x);
   const double roi_closest_x = (roi_left_dist <= roi_right_dist) ? roi.left() : roi.right();
   const double roi_guide_length = std::abs(image_mid_x - roi_closest_x);
   const int horizontal_guide_thickness = std::clamp<int>(static_cast<int>(roi_guide_length / 8.0), 2, 25);
   pen.setColor(QColor(255, 140, 0));
   pen.setWidth(horizontal_guide_thickness);
   pen.setCosmetic(false);
   painter.setPen(pen);
   painter.drawLine(QPointF(roi_closest_x, roi_mid_y), QPointF(image_mid_x, roi_mid_y));


   label->setPixmap(QPixmap::fromImage(resized));
}


void SpecificWorker::draw_lidar(const auto &points, QGraphicsScene* scene)
{
  static std::vector<QGraphicsItem*> draw_points;
  for (const auto &p : draw_points)
  {
     scene->removeItem(p);
     delete p;
  }
  draw_points.clear();


  const QColor color("LightGreen");
  const QPen pen(color, 10);
  //const QBrush brush(color, Qt::SolidPattern);
  for (const auto &p : points)
  {
     const auto dp = scene->addRect(-25, -25, 50, 50, pen);
     dp->setPos(p.x, p.y);
     draw_points.push_back(dp);   // add to the list of points to be deleted next time
  }
}

void SpecificWorker::draw_person_in_lidar(float person_angle, float distance, QGraphicsScene* scene)
{
   static std::vector<QGraphicsItem*> draw_points;
   for (const auto &p : draw_points)
   {
     scene->removeItem(p);
     delete p;
   }
   draw_points.clear();


   const QColor color(Qt::red);
   const QPen pen(color, 50);
  
   auto circ = scene->addEllipse(-100, -100, 200, 200, pen);
   // compute center from angle and dist
   auto cx = distance * sin(person_angle+0.1);
   auto cy = distance * cos(person_angle);
   QPointF center{cx, cy};
   circ->setPos(center);
   draw_points.push_back(circ);   


   auto line = scene->addLine(0, 0, center.x(), center.y(), pen);
   line->setZValue(-1); // Ensure the line is behind the person polygon
   draw_points.push_back(line);


   // Add text label with distance and angle
   double angle_deg = person_angle * (180.0 / M_PI);
   QString labelText = QString("Target Person\nDist: %1mm\nAng: %2°")
                           .arg(distance, 0, 'f', 2)
                           .arg(angle_deg, 0, 'f', 1);


   static QGraphicsTextItem* m_personLabel = nullptr; // Persistent label across calls
   // Create or Update the Label
   if (m_personLabel== nullptr)
   {
       // Create the item once and add it to the scene
       m_personLabel = scene->addText(labelText);
      
       // --- FIX FOR FLIPPED Y-AXIS ---
       // This cancels out the scene's scale(1, -1) transform
       m_personLabel->setTransform(QTransform().scale(1, -1));
       // -----------------------------


       // Set a visible style
       QFont font = m_personLabel->font();
       font.setPixelSize(100);
       //font.setBold(true);
       m_personLabel->setFont(font);
       m_personLabel->setDefaultTextColor(Qt::blue);
      
       // Add a background box for readability (optional but recommended)
       // Since QGraphicsTextItem doesn't have a background, we wrap it in a GraphicsItem
       // or just set a semi-transparent background via a separate rectangle if needed.
   } else     // Update existing item's text
       m_personLabel->setPlainText(labelText);
  


   // Update Position (slightly offset from the point)
   m_personLabel->setPos(center.x() + 10, center.y() - 10);
  
   // Ensure the label is always on top of other Lidar points
   m_personLabel->setZValue(100);
}

std::optional<std::tuple<std::vector<QPointF>, float>> SpecificWorker::project_lidar_to_camera(
                                           const RoboCompLidar3D::TPoints& points,
                                           const Eigen::Isometry3d& cam_transform,
                                           double personAngle,
                                           const RoboCompCamera360RGB::TImage &image,
                                           const QRect &roi,
                                           double azimuth_sign,
                                           double azimuth_offset)
{


   // filter points based on the specified angle range and distance
   auto filtered_cone = cono(points, personAngle - 0.3, personAngle + 0.3, 0.0f);
   if(filtered_cone.empty()) { qWarning() << "Empty cone"; return {};};

   std::vector<QPointF> projected_points;
   projected_points.reserve(points.size());
   std::vector<float> distances;


   for (const auto& lp : filtered_cone)
   {
       // Convert Lidar point to Eigen vector
       Eigen::Vector3d p_lidar(lp.x, lp.y, lp.z);


       // Transform point to Camera Local Frame
       // P_cam = R_inv * (P_lidar - t)
       Eigen::Vector3d p_cam = cam_transform * p_lidar;


       // Equirectangular Projection
       const double r = p_cam.norm();
       if (r < 1e-9)
       {
           projected_points.emplace_back(image.width / 2.0, image.height / 2.0);
           continue;
       }


       // Azimuth (u)
       double u = ((azimuth_sign * std::atan2(p_cam.x(), p_cam.y()) + azimuth_offset) / (2.0 * M_PI) + 0.5) * image.width;
       u = std::fmod(u, static_cast<double>(image.width));
       if (u < 0.0) u += image.width;


       // Elevation (v)
       const double v = (std::asin(-p_cam.z() / r) / M_PI + 0.5) * image.height;


       // Check if the projected point is within the person's bounding box
       if (roi.contains(QPoint(u, v)))
       {
           projected_points.emplace_back(u, v);
           distances.push_back(r);
       }
   }
   // compute the minimal depth of the projected points within the bounding box
   if(distances.empty()) return {};
   float measured_depth = *std::ranges::min_element(distances);


   return std::make_tuple(projected_points, measured_depth);
}

//------------------------------------------------------

RoboCompLidar3D::TPoints SpecificWorker::cono(const RoboCompLidar3D::TPoints &points, float minAngle, float maxAngle, float distance)
{
	RoboCompLidar3D::TPoints filtered_points;
	for (const auto &point : points)
	{
		bool inAngle= (point.phi > minAngle) and (point.phi < maxAngle);
		if (not inAngle)
			continue;
		if(distance > 0)
		{
			if (point.distance2d >= distance ||
				point.distance2d <= this->ROBOT_LENGTH / 2.1)
				continue;
		}
		filtered_points.push_back(point);
	}
	return filtered_points;
}

void SpecificWorker::move_robot(float adv, float rot)
{
	
	try{ omnirobot_proxy->setSpeedBase(0.0f, adv, rot);} 
	catch(const Ice::Exception& e){std::cerr << e.what() << '\n';	};		
}

std::tuple<SpecificWorker::STATE, float, float> SpecificWorker::navigation(const RoboCompLidar3D::TPoints &points){
	try
	{

		std::tuple<STATE, float, float> result(state, 1000.0f, 0.0f);	//State -> enum class



		switch(state)
		{
			case STATE::IDLE:{
				state = STATE::FORWARD;
				break;
			}

			case STATE::FORWARD:
			{
				auto con = cono(points, -0.3f, 0.3f, 0.0f);
				if(auto min = std::ranges::min_element(con, [](auto &a, auto &b){ return a.distance2d < b.distance2d;}); min != con.end())
				// std::cout<< min->distance2d<<std::endl;
					if(min->distance2d < SAFETY_THRESHOLD)  // COLLISION
					{
						//move_robot(0.f, 0.f);
						//result = (STATE::TURN, 0.0f, 0.0f);
						state = STATE::TURN;
						std::get<1>(result) = 0.0f; 
						//return;
					}
					else if (min->distance2d < SAFETY_THRESHOLD * 1.66)
					{
						std::get<1>(result) = 500.0f; //Velocidad x
						std::get<2>(result) = 1.0f; //Rotacion del robot
					}
					else
						state = STATE::SPIRAL;
					
					
				//move_robot(700.f, 0.f);
				//result = (1000.f, 0.f);
				break;
			}			
				
			case STATE::TURN:
			{
				std::get<1>(result) = 0.0f; 
				auto con = cono(points, -0.3f, 0.3f, 0.0f);
				if(auto min = std::ranges::min_element(con, [](auto &a, auto &b){ return a.distance2d < b.distance2d;}); min != con.end())
					if (min->distance2d < SAFETY_THRESHOLD)  // COLLISION
					{
						//move_robot(0.f, 500.f);
						std::get<2>(result) = 2.7f; 
						//return;
					}
					else
					{
						state = STATE::FORWARD;
					}
					
				break;
			}
				
			case STATE::FOLLOW_WALL:{

				break;
			}

			case STATE::SPIRAL:{ 
				
				auto con = cono(points, -0.3f, 0.3f, 0.0f);
				if(auto min = std::ranges::min_element(con, [](auto &a, auto &b){ return a.distance2d < b.distance2d;}); min != con.end())
					if (min->distance2d < SAFETY_THRESHOLD * 2)  // COLLISION
					{
						std::cout << "spiral" << std::endl;
						//move_robot(0.f, 500.f);
						std::get<1>(result) = 500.0f; 
						state = STATE::FORWARD;
						//return;
					}
				break;
			}
		}
		//state = std::get<0>(result);
		std::get<0>(result) = state;

		return result;
	}
	catch(const Ice::Exception& e){
		std::cerr << e.what() << '\n';
	};
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////




/*
void SpecificWorker::draw_lidar(const auto &points, QGraphicsScene* scene)
{
   static std::vector<QGraphicsItem*> draw_points;
   for (const auto &p : draw_points)
   {
      scene->removeItem(p);
      delete p;
   }

   draw_points.clear();

   auto ppoints = cono(points, -10.f, 10.f, 0);
   qInfo() << "DIST" <<  ppoints.front().distance2d << ppoints.front().r;

   const QColor color("LightGreen");
   const QPen pen(color, 10);
   //const QBrush brush(color, Qt::SolidPattern);
   for (const auto &p : ppoints)
   {
      const auto dp = scene->addRect(-25, -25, 50, 50, pen);
      dp->setPos(p.x, p.y);
      draw_points.push_back(dp);   // add to the list of points to be deleted next time
   }
}
*/


void SpecificWorker::new_target_slot(QPointF p){
	qInfo() << p;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::emergency()
{
    fps.print("Emergency worker", 3000);
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}


//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}



/**************************************/
// From the RoboCompCamera360RGB you can call this methods:
// RoboCompCamera360RGB::TImage this->camera360rgb_proxy->getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)

/**************************************/
// From the RoboCompCamera360RGB you can use this types:
// RoboCompCamera360RGB::TRoi
// RoboCompCamera360RGB::TImage

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TColorCloudData this->lidar3d_proxy->getColorCloudData()
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarData(string name, float start, float len, int decimationDegreeFactor)
// RoboCompLidar3D::TDataImage this->lidar3d_proxy->getLidarDataArrayProyectedInImage(string name)
// RoboCompLidar3D::TDataCategory this->lidar3d_proxy->getLidarDataByCategory(TCategories categories, long timestamp)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataProyectedInImage(string name)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataWithThreshold2d(string name, float distance, int decimationDegreeFactor)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// RoboCompOmniRobot::void this->omnirobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->resetOdometer()
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->setSpeedBase(float advx, float advz, float rot)
// RoboCompOmniRobot::void this->omnirobot_proxy->stopBase()

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

/**************************************/
// From the RoboCompVisualElements you can call this methods:
// RoboCompVisualElements::TObjects this->visualelements_proxy->getVisualObjects(TObjects objects)
// RoboCompVisualElements::void this->visualelements_proxy->setVisualObjects(TObjects objects)

/**************************************/
// From the RoboCompVisualElements you can use this types:
// RoboCompVisualElements::TRoi
// RoboCompVisualElements::TObject
// RoboCompVisualElements::TObjects
