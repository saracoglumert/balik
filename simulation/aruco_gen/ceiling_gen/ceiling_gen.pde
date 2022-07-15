//each pixel is 1mm.
int numid=20;
int numrow=4;
int numcol=3;
int alpha=140;
int beta=140;
int tagsize=18;
boolean done=false;

PImage[] imgs = new PImage[numid+1];

//need 90 CCW

void setup() {
  size(1070,730);
  for(int i=0;i<numid;i++){
    imgs[i]=loadImage("files/marker_".concat(str(i)).concat(".jpg"));
  }
}
  //  image(imgs[i], i*width/10, i*height/10);
void draw() {
  if (!done) {
    for(int row=0; row<numrow; row++) {
      for(int col=0; col<numcol; col++) {
        int index=(3*row)+col;
        if(index<numid){
          image(imgs[index], beta*(row+1)-(tagsize/2), alpha*(col+1)-(tagsize/2));
        }
      }
    }
    save("ceilingbase.png");
  }
  done=true;
}
