/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "CameraFactory.h"

boost::shared_ptr< CameraFactory > CameraFactory::m_instance;

CameraFactory::CameraFactory( ) 
{

}

boost::shared_ptr< CameraFactory > CameraFactory::instance( void )
{
    if ( m_instance.get( ) == 0 )
    {
        m_instance.reset( new CameraFactory );
    }
    return m_instance;
}

CameraPtr CameraFactory::generateCameraFromYamlFile( const std::string& filename )
{
    cv::FileStorage fs( filename, cv::FileStorage::READ );
    Camera::ModelType modelType;
    std::string sModelType;
    fs["model_type"] >> sModelType;
    if ( boost::iequals( sModelType, "pinhole" ) )
    {
        modelType = Camera::PINHOLE;
    }
    switch ( modelType )
    {
        case Camera::PINHOLE:
        {
            PinholeCameraPtr camera( new PinholeCamera );
            PinholeCamera::Parameters params = camera->getParameters( );
            params.readFromYamlFile( filename );
            camera->setParameters( params );
            return camera;
        }
    }
    return CameraPtr( );
}