int numid=10;
int numrow=4;
int numcol=4;
int padding=140;
boolean done=false;

PImage[] imgs = new PImage[numid];



void setup() {
  size(1070,730);
  //imgs[0]=loadImage("files/marker_0.jpg");
  for(int i=0;i<numid;i++){
    imgs[i]=loadImage("files/marker_".concat(str(i)).concat(".jpg"));
  }
}
  //  image(imgs[i], i*width/10, i*height/10);
void draw() {
  if (!done) {
    for(int row=0; row<numrow; row++) {
      for(int col=0; col<numcol; col++) {
        int index=row+col;
        if(index<numid){
          image(imgs[index], padding/2+row*((width)/numrow-2), padding/2+col*((height)/numcol-2));
        }
      }
    }
    save("ceiling.png");
  }
  done=true;
}
