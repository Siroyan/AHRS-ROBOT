#include <math.h>
void measure_gps(){
  // 1つのセンテンスを読み込む
  String line = gpsSerial.readStringUntil('\n');
  if(line != ""){
    int i, index = 0, len = line.length();
    String str = "";
  
    // StringListの生成(簡易)
    String list[30];
    for (i = 0; i < 30; i++) {
      list[i] = "";
    }
    // 「,」を区切り文字として文字列を配列にする
    for (i = 0; i < len; i++) {
      if (line[i] == ',') {
        list[index++] = str;
        //index++;
        str = "";
        continue;
      }
      str += line[i];
    }
    // $GPGGAセンテンスのみ読み込む
    if (list[0] == "$GPGGA") {
      ido = NMEA2DD(list[2].toFloat());
      keido = NMEA2DD(list[4].toFloat());
    }
  }
}


float NMEA2DD(float val) {
  /* NMEAの緯度経度を「度」(DD)の文字列に変換する */
  int d = val / 100;
  int m = (((val / 100.0) - d) * 100.0) / 60;
  float s = (((((val / 100.0) - d) * 100.0) - m) * 60) / (60 * 60);
  return float (d + m + s);
}

const float R = 6378137;

float distance(float x_1,float y_1,float x_2,float y_2){
  /* ゴールまでの距離を返す */
  float delta_x = x_2 - x_1;
  float delta_y = y_2 - y_1;
  float d = R * sqrt(delta_x * delta_x + delta_y * delta_y);
  return d;
}

float azimuth(float x_1,float y_1,float x_2,float y_2){
  /* 緯度経度より、自機位置とゴールの位置の方位角を計算 */
  float delta_x = x_2 - x_1;
  float delta_y = y_2 - y_1;
  float phi = rad2deg(atan2(R * delta_y, R * delta_x));
  /* atan2はx軸からの角度(つまり東を0)なので、北へ90度ズラす */
  if(-90 <= phi && phi <= 180) phi = phi - 90;
  if(-180 <= phi && phi < -90) phi = phi + 270;
  return phi;
}
