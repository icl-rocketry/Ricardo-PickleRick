#include "estimator.h"

#include <string>
#include "math.h"

#include "Eigen/Eigen"

#include <ArduinoJson.h>
#include <libriccore/riccorelogging.h>
#include <librrc/Helpers/jsonconfighelper.h>

#include "Config/types.h"
#include "Config/systemflags_config.h"

#include "sensors.h"

Estimator::Estimator(Types::CoreTypes::SystemStatus_t &systemstatus) : _systemstatus(systemstatus),
                                                                       update_frequency(2000), // 500Hz update
                                                                       _homeSet(false),
                                                                       madgwick(0.5f, 0.005f), // beta | gyroscope sample time step (s)
                                                                       refOrientation(defaultOrientation)
                                                                       {};

void Estimator::setup()
{
   resetLocalization();
   resetOrientation();
};

void Estimator::configure(JsonObjectConst conf)
{
   // update board orientation this is applied when converthing back to sensor frame where the orientaiton of sensor matters
   // upside down should be retireved from config file
   setRefOrientation(conf["Orientation"]);
}


void Estimator::update(const SensorStructs::raw_measurements_t &raw_sensors)
{

   unsigned long dt = (unsigned long)(micros() - last_update); // explictly casting to prevent micros() overflow cuasing issues

   if (dt > update_frequency)
   {

      last_update = micros();                   // update last_update
      float dt_seconds = float(dt) * 0.000001F; // conversion to seconds

      if (_systemstatus.flagSetOr(SYSTEM_FLAG::ERROR_IMU))
      {

         if (_systemstatus.flagSetOr(SYSTEM_FLAG::ERROR_GPS) && _systemstatus.flagSetOr(SYSTEM_FLAG::ERROR_BARO))
         {
            // no data so we cant calculate any nav solution
            changeEstimatorState(ESTIMATOR_STATE::NOSOLUTION, "no data, cannot compute navigation solution");
            return;
         }

         if (_systemstatus.flagSetOr(SYSTEM_FLAG::ERROR_GPS))
         {
            // baro only update
            // TODO - add a z velocity estimate using first order filter and disrete derivative

            if (_homeSet) // if no home, this falls thru to the no home
            {
               
               baroUpdate(raw_sensors.baro.alt);
               changeEstimatorState(ESTIMATOR_STATE::PARTIAL_NO_IMU_NO_GPS, "no IMU and GPS");
               predictLocalizationKF(dt_seconds);
               return;
            }
         }

         if (_systemstatus.flagSetOr(SYSTEM_FLAG::ERROR_BARO))
         {
            // gps only update, no fusion as filtering this data will only result in a worse solution
            if (_homeSet) // if false, this falls thru to the more important error which is no home set
            {
               state.position = localizationkf.GPStoNED(raw_sensors.gps.lat,
                                                        raw_sensors.gps.lng,
                                                        raw_sensors.gps.alt);
               state.velocity = Eigen::Vector3f{raw_sensors.gps.v_n / 1000.0f,
                                                raw_sensors.gps.v_e / 1000.0f,
                                                raw_sensors.gps.v_d / 1000.0f};
               changeEstimatorState(ESTIMATOR_STATE::PARTIAL_NO_IMU_NO_BARO, "no IMU and BARO, raw gps navigation solution");
               return;
            }
         }
      }
      else
      { // we have imu so calculate orientation and update localizationkf
         if (_systemstatus.flagSetOr(SYSTEM_FLAG::ERROR_MAG))
         {
            computeOrientation(raw_sensors.accelgyro.gx, raw_sensors.accelgyro.gy, raw_sensors.accelgyro.gz,
                               raw_sensors.accelgyro.ax, raw_sensors.accelgyro.ay, raw_sensors.accelgyro.az,
                               dt_seconds);
            // check that there isnt a bigger error
            if (_homeSet)
            {
               changeEstimatorState(ESTIMATOR_STATE::PARTIAL_NO_MAG, "no mag, heading unreliable");
            }
         }
         else
         {
            computeOrientation(raw_sensors.accelgyro.gx, raw_sensors.accelgyro.gy, raw_sensors.accelgyro.gz,
                               raw_sensors.accelgyro.ax, raw_sensors.accelgyro.ay, raw_sensors.accelgyro.az,
                               raw_sensors.mag.mx, raw_sensors.mag.my, raw_sensors.mag.mz,
                               dt_seconds);
         }
         // transform angular rates from body frame to earth frame
         updateAngularRates(raw_sensors.accelgyro.gx, raw_sensors.accelgyro.gy, raw_sensors.accelgyro.gz);
         localizationkf.accelUpdate(getLinearAcceleration(raw_sensors.accelgyro.ax, raw_sensors.accelgyro.ay, raw_sensors.accelgyro.az) * g);
         // only update with high-g accelerometer if the low g acceleromter is working
         if (!_systemstatus.flagSetOr(SYSTEM_FLAG::ERROR_HACCEL))
         {
            // localizationkf.HaccelUpdate(getLinearAcceleration(raw_sensors.accel.ax, raw_sensors.accel.ay, raw_sensors.accel.az) * g);
         }
      }

      if (!_homeSet)
      {
         changeEstimatorState(ESTIMATOR_STATE::NO_HOME, "Home not set, Localization not avaliable");
         return;
      }

      if (_systemstatus.flagSetOr(SYSTEM_FLAG::ERROR_GPS))
      {
         if (_systemstatus.flagSetOr(SYSTEM_FLAG::ERROR_BARO))
         {
            // no data so only orientation avalibale
            changeEstimatorState(ESTIMATOR_STATE::PARTIAL_NO_GPS_NO_BARO, "no gps and baro, position and velocity estimates unreliable!");
            predictLocalizationKF(dt_seconds);
            return;
         }
         // baro only update
         changeEstimatorState(ESTIMATOR_STATE::PARTIAL_NO_GPS, "no gps, N and E positon and velocity estimates unreliable!");
      }
      else
      {
         // if there is no error in the gps check if gps data is updated
         if (raw_sensors.gps.updated)
         {
            localizationkf.gpsUpdate(raw_sensors.gps.lat,
                                     raw_sensors.gps.lng,
                                     raw_sensors.gps.alt,
                                     raw_sensors.gps.v_n,
                                     raw_sensors.gps.v_e,
                                     raw_sensors.gps.v_d);
         }
      }

      if (_systemstatus.flagSetOr(SYSTEM_FLAG::ERROR_BARO))
      {
         changeEstimatorState(ESTIMATOR_STATE::PARTIAL_NO_BARO, "no baro");
      }
      else
      {
         baroUpdate(raw_sensors.baro.alt);
      }

      predictLocalizationKF(dt_seconds);

      if (!_systemstatus.flagSetOr(SYSTEM_FLAG::ERROR_IMU, SYSTEM_FLAG::ERROR_GPS, SYSTEM_FLAG::ERROR_BARO, SYSTEM_FLAG::ERROR_MAG, SYSTEM_FLAG::ERROR_HACCEL))
      {
         // if there are no errors with sensors then the estimator state must be nominal
         changeEstimatorState(ESTIMATOR_STATE::NOMINAL, "all data avaliable, full update");
      }
   }
};

void Estimator::setHome(const SensorStructs::raw_measurements_t &raw_sensors)
{
   // record current gps coordinates as home
   state.gps_launch_lat = raw_sensors.gps.lat;
   state.gps_launch_long = raw_sensors.gps.lng;
   state.gps_launch_alt = raw_sensors.gps.alt;
   // update barometer reference altitude
   state.baro_ref_alt = raw_sensors.baro.alt;
   // log the new home position
   RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Home Position Updated to Lat: " + std::to_string(state.gps_launch_lat) + " Long: " + std::to_string(state.gps_launch_long) + " Alt: " + std::to_string(state.gps_launch_alt) + " Baro Ref Alt: " + std::to_string(state.baro_ref_alt));
   // update reference coordinates in position estimation
   localizationkf.updateGPSReference(state.gps_launch_lat, state.gps_launch_long, state.gps_launch_alt);
   localizationkf.reset(); // reinitialize the filter
   _homeSet = true;
}

void Estimator::computeOrientation(const float &gx, const float &gy, const float &gz,
                                   const float &ax, const float &ay, const float &az,
                                   const float &mx, const float &my, const float &mz, float dt)
{

   // calculate orientation solution
   madgwick.setDeltaT(dt); // update integration time

   madgwick.update(gx, gy, gz, ax, ay, az, mx, my, mz);
   // madgwick.update(gx, gy, gz, ax, ay, az-2, mx, my, mz);

   updateOrientation();
}

void Estimator::computeOrientation(const float &gx, const float &gy, const float &gz,
                                   const float &ax, const float &ay, const float &az, float dt)
{

   // calculate orientation solution
   madgwick.setDeltaT(dt); // update integration time

   madgwick.updateIMU(gx, gy, gz, ax, ay, az);
   // update orientation
   updateOrientation();
}

void Estimator::updateOrientation()
{

   // update orientation
   state.orientation = madgwick.getOrientation();
   state.eulerAngles = madgwick.getEulerAngles();

   //TODO finish this
   // state.rocketOrientation = (state.orientation * refOrientation).normalized();
   // state.rocketEulerAngles = {};
   // state.rocketOrientation = {};

   state.tilt = calculateNutation(state.eulerAngles);
}

void Estimator::updateAngularRates(const float &gx, const float &gy, const float &gz)
{
   madgwick.getInverseRotationMatrix() * Eigen::Vector3f{gx, gy, gz}; // this might be incorrect?
}

Eigen::Vector3f Estimator::getLinearAcceleration(const float &ax, const float &ay, const float &az)
{
   return (madgwick.getRotationMatrix() * Eigen::Vector3f{ax, ay, az}) + Eigen::Vector3f{0, 0, 1};
};

void Estimator::changeEstimatorState(ESTIMATOR_STATE status, std::string logmessage)
{
   if (state.estimator_state != static_cast<uint8_t>(status))
   { // check if we already have logged this
      state.estimator_state = static_cast<uint8_t>(status);
      if (status != ESTIMATOR_STATE::NOMINAL)
      {
         _systemstatus.newFlag(SYSTEM_FLAG::ERROR_ESTIMATOR, logmessage);
      }
      else if (_systemstatus.flagSetOr(SYSTEM_FLAG::ERROR_ESTIMATOR))
      {
         _systemstatus.deleteFlag(SYSTEM_FLAG::ERROR_ESTIMATOR, logmessage);
      }
   }
}

void Estimator::changeBeta(float beta)
{
   madgwick.setBeta(beta);
}

void Estimator::resetOrientation()
{
   madgwick.reset();
   RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Orientation Reset");
}

void Estimator::resetLocalization()
{
   localizationkf.reset();
   RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Estimator Reset");
}

void Estimator::setIgnitionTime(uint32_t time)
{
   state.ignitionTime = time;
   RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Ignition commanded at " + std::to_string(time));
}

void Estimator::setLiftoffTime(uint32_t time)
{
   state.liftoffTime = time;
   RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Liftoff detected at " + std::to_string(time));
}

void Estimator::setApogeeTime(uint32_t time)
{
   state.apogeeTime = time;
   RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Apogee detected at " + std::to_string(time));
}

const SensorStructs::state_t &Estimator::getData()
{
   // again as with sensors, this should be updated to return the data in a threadsafe manner
   return state;
}

void Estimator::predictLocalizationKF(const float &dt)
{
   localizationkf.predict(dt);
   state.acceleration = localizationkf.getAcceleration();
   state.velocity = localizationkf.getVelocity();
   state.position = localizationkf.getPosition();
}

float Estimator::calculateNutation(const Eigen::Vector3f &euler)
{
   //domain of acos is [0,pi] -> tilt angle will always be an absolute value 
   return acos(cos(euler(1)) * cos(euler(2)));
}

void Estimator::baroUpdate(const float& altitude)
{
   if (altitude > BARO_MAX_ALT)
   {
      return;
   }
   localizationkf.baroUpdate(altitude - state.baro_ref_alt);
}

void Estimator::setRefOrientation(JsonObjectConst conf)
{
   using namespace LIBRRC::JsonConfigHelper;
   std::string type = getIfContains<std::string>(conf,"type","");
   try
   {
      if (type == "")
      {
         RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("No Orientation Type Given, using Default!");
         return;
      }
      else if (type == "euler") //extepd format r,p,y
      {
         float roll = getIfContains<float>(conf,"roll");
         float pitch = getIfContains<float>(conf,"pitch");
         float yaw = getIfContains<float>(conf,"yaw");

      }
      else if (type == "quaternion") //exepcetd format w,x,y,z
      {
         refOrientation = Eigen::Quaternionf({getIfContains<float>(conf,"w"),
                                              getIfContains<float>(conf,"x"),
                                              getIfContains<float>(conf,"y"),
                                              getIfContains<float>(conf,"z")});
         refOrientation.normalize(); //ensure ref orinetation is normalized
      }
      else if (type == "axisAlign")
      {
         int x_axis = getIfContains<int>(conf,"x_axis");
         int y_axis = getIfContains<int>(conf,"y_axis");
         int z_axis = getIfContains<int>(conf,"z_axis");
         int x_inv = getIfContains<int>(conf,"x_inv") ? -1 : 1;
         int y_inv = getIfContains<int>(conf,"y_inv") ? -1 : 1;
         int z_inv = getIfContains<int>(conf,"z_inv") ? -1 : 1;

      }
      else
      {
         RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Illegal Orientation Type Given, using Default!");
         return;
      }
   }
   catch (const std::exception &e)
   {
      RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Exception occured while processing orientation! - " + std::string(e.what()));

      return;
   }

   RicCoreLogging::log<RicCoreLoggingConfig::LOGGERS::SYS>("Ref Orientation updated to:" );

}