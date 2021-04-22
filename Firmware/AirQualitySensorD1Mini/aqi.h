/**
   Use the latest sensor readings to calculate the Air Quality
   Index value using the UK reporting method.
*/
void calculateUkAqi()
{
  uint8_t pm2p5_aqi = 0;
  if (g_pm2p5_ppd_value <= 11) {
    pm2p5_aqi = 1;
  } else if (g_pm2p5_ppd_value <= 23) {
    pm2p5_aqi = 2;
  } else if (g_pm2p5_ppd_value <= 35) {
    pm2p5_aqi = 3;
  } else if (g_pm2p5_ppd_value <= 41) {
    pm2p5_aqi = 4;
  } else if (g_pm2p5_ppd_value <= 47) {
    pm2p5_aqi = 5;
  } else if (g_pm2p5_ppd_value <= 53) {
    pm2p5_aqi = 6;
  } else if (g_pm2p5_ppd_value <= 58) {
    pm2p5_aqi = 7;
  } else if (g_pm2p5_ppd_value <= 64) {
    pm2p5_aqi = 8;
  } else if (g_pm2p5_ppd_value <= 70) {
    pm2p5_aqi = 9;
  } else {
    pm2p5_aqi = 10;
  }

  uint8_t pm10p0_aqi = 0;
  if (g_pm10p0_ppd_value <= 16) {
    pm10p0_aqi = 1;
  } else if (g_pm10p0_ppd_value <= 33) {
    pm10p0_aqi = 2;
  } else if (g_pm10p0_ppd_value <= 50) {
    pm10p0_aqi = 3;
  } else if (g_pm10p0_ppd_value <= 58) {
    pm10p0_aqi = 4;
  } else if (g_pm10p0_ppd_value <= 66) {
    pm10p0_aqi = 5;
  } else if (g_pm10p0_ppd_value <= 75) {
    pm10p0_aqi = 6;
  } else if (g_pm10p0_ppd_value <= 83) {
    pm10p0_aqi = 7;
  } else if (g_pm10p0_ppd_value <= 91) {
    pm10p0_aqi = 8;
  } else if (g_pm10p0_ppd_value <= 100) {
    pm10p0_aqi = 9;
  } else {
    pm10p0_aqi = 10;
  }

  if (pm10p0_aqi > pm2p5_aqi)
  {
    g_uk_aqi_value = pm10p0_aqi;
  } else {
    g_uk_aqi_value = pm2p5_aqi;
  }
}
