
void drawMainFace(uint16_t Color, uint16_t bg, uint16_t HR, uint16_t MIN, uint16_t SEC, uint16_t DD, uint16_t MM, uint16_t YR){
//128 x 160 screen

}

void DrawThickRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t wallwdth, uint16_t color) {
  for(uint16_t i = 0; i < wallwdth; i++){
    drawRectWH(x + i, y + i, w - i * 2, h - i * 2, color);
  }
}
void DrawCirclePartial(uint16_t cx, uint16_t cy, uint16_t r, uint16_t thc, float_t percentdec, uint16_t color) {
  #define PI 3.14159265358979323846
  float step =  (2*PI)/(2*PI*r);
  for(double t = 0; t <= (2 * PI)*percentdec; t += step){
    for(int i = 0; i <= thc; i +=1){
        double x = (r+i) * cos(t);
        double y = (r+i) * sin(t);
        drawPixel(x+cx,y+cy,color);
    }
  }
}