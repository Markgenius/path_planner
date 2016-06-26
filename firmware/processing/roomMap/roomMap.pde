int hallPoints[][] = new int[17][2];
int centerPoints[][] = new int[7][2];
int paths[][] = new int[27][2];
int pointValue[] = new int[208];
BufferedReader reader;
String line = "a";
int startPoint[] = {0, 0};
int endPoint[] = {0, 0};
int startPointID = 0, endPointID=0;
boolean initialClick = true;
PImage bg;
int[] route = new int[28];
String[] command = new String[28];
int lastDir=-90;

void setup() {
  scale(0.5);
  size(960, 720);
  bg = loadImage("map.jpg");
  bg.resize(960, 720);
  background(bg);
  frameRate(5);
  reader = createReader("roommap.txt"); 

  while (line != null) {
    try {
      line = reader.readLine();
    } 
    catch (IOException e) {
      e.printStackTrace();
      line = null;
    }
    if (line == null || line.indexOf(" ")==-1) continue;
    String[] pieces = split(line, ' ');
    switch(pieces[0].charAt(0)) {
    case '1':
      int id1 = int(pieces[0].substring(1))-1;
      hallPoints[id1][0] = int(pieces[1]);
      hallPoints[id1][1] = int(pieces[2]);
      break;
    case '2':
      int id2 = int(pieces[0].substring(1))-1;
      centerPoints[id2][0] = int(pieces[1]);
      centerPoints[id2][1] = int(pieces[2]);
      break;
    case '3':
      int id3 = int(pieces[0].substring(1))-1;
      paths[id3][0] = int(pieces[1]);
      paths[id3][1] = int(pieces[2]);
      break;
    default:
      break;
    }
  }
  text("", 0, 0);
}

void draw() {
  scale(0.8);
}

void mousePressed() {
  if (startPoint[0]==0 && startPoint[1]==0) {
    background(bg);
    for (int i=0; i<hallPoints.length; i++) {
      if (dist(mouseX/0.8, mouseY/0.8, hallPoints[i][0], hallPoints[i][1])<=30) {
        startPoint[0] = hallPoints[i][0];
        startPoint[1] = hallPoints[i][1];
        startPointID = 101+i;
      }
    }
    for (int j=0; j<centerPoints.length; j++) {
      if (dist(mouseX/0.8, mouseY/0.8, centerPoints[j][0], centerPoints[j][1])<=40) {
        startPoint[0] = centerPoints[j][0];
        startPoint[1] = centerPoints[j][1];
        startPointID = 201+j;
      }
    }
    if (startPoint[0]==0 && startPoint[1]==0) return;
    fill(255, 0, 0);
    ellipse(startPoint[0], startPoint[1], 15, 15);
    initialClick = false;
  } else {
    background(bg);
    if(endPoint[0]!=0 || endPoint[1]!=0){
      startPoint[0]=endPoint[0];
      startPoint[1]=endPoint[1];
      startPointID=endPointID;
    }
    fill(255,255,0);
    ellipse(startPoint[0], startPoint[1], 15, 15);
    for (int i=0; i<hallPoints.length; i++) {
      if (dist(mouseX/0.8, mouseY/0.8, hallPoints[i][0], hallPoints[i][1])<=30) {
        endPoint[0] = hallPoints[i][0];
        endPoint[1] = hallPoints[i][1];
        endPointID = 101+i;
      }
    }
    for (int j=0; j<centerPoints.length; j++) {
      if (dist(mouseX/0.8, mouseY/0.8, centerPoints[j][0], centerPoints[j][1])<=40) {
        endPoint[0] = centerPoints[j][0];
        endPoint[1] = centerPoints[j][1];
        endPointID = 201+j;
      }
    }
    if (endPoint[0]==0 && endPoint[1]==0) return;
    fill(255, 0, 0);
    ellipse(endPoint[0], endPoint[1], 15, 15);
    findRoute();
    drawRoute();
    initialClick = true;
  }
}

void keyPressed() {
  if (keyCode==ENTER) {
    println(getCommand());
  }
  
}

int[] findRoute() {
  route = new int[28];
  int tempRoute[] = new int[28];
  for (int i=101; i<118; i++) {
    pointValue[i] = 1000000;
  }
  for (int i=201; i<208; i++) {
    pointValue[i] = 1000000;
  }
  pointValue[startPointID] = 0;
  updateNeighbourPointValue(startPointID);
  for (int i=101; i<118; i++) {
    fill(0, 0, 255);
    textSize(20);
    textAlign(LEFT, TOP);
    text(pointValue[i], pointID2cdn(i)[0]+10, pointID2cdn(i)[1]+10);
  }
  for (int i=201; i<208; i++) {
    fill(0, 0, 255);
    textSize(20);
    textAlign(LEFT, TOP);
    text(pointValue[i], pointID2cdn(i)[0]+15, pointID2cdn(i)[1]+15);
  }
  int pathCurrent = 0;
  tempRoute[0] = endPointID;
  boolean breakOuterFor = false;
  for (int pt=0; pt<28; pt++) {
    boolean breakInnerFor = false;
    for (int i=0; i<paths.length&&!breakInnerFor; i++) {
      int linkCdn1[] = pointID2cdn(paths[i][0]);
      int linkCdn2[] = pointID2cdn(paths[i][1]);
      int startCdn[] = pointID2cdn(tempRoute[pathCurrent]);
      if (startCdn[0] == linkCdn1[0] && startCdn[1] == linkCdn1[1]) {
        if (pointValue[paths[i][1]]==pointValue[tempRoute[pathCurrent]]-linkLength(301+i)) {
          pathCurrent++;
          tempRoute[pathCurrent] = paths[i][1];
          breakInnerFor =true;
          //if (paths[i][1] == startPointID)breakOuterFor = true;
        }
      }
      if (startCdn[0] == linkCdn2[0] && startCdn[1] == linkCdn2[1]) {
        if (pointValue[paths[i][0]]==pointValue[tempRoute[pathCurrent]]-linkLength(301+i)) {
          pathCurrent++;
          tempRoute[pathCurrent] = paths[i][0];
          breakInnerFor =true;
          //if (paths[i][0] == startPointID)breakOuterFor = true;
        }
      }
    }
  }
  for (int i=0; i<=pathCurrent; i++) {
    route[i] = tempRoute[pathCurrent-i];
  }
  return route;
}
void updateNeighbourPointValue(int pointID) {
  for (int i=0; i<paths.length; i++) {
    int linkCdn1[] = pointID2cdn(paths[i][0]);
    int linkCdn2[] = pointID2cdn(paths[i][1]);
    int startCdn[] = pointID2cdn(pointID);
    if (startCdn[0] == linkCdn1[0] && startCdn[1] == linkCdn1[1]) {
      if (pointValue[paths[i][1]]>pointValue[pointID]+linkLength(301+i)) {
        pointValue[paths[i][1]] = pointValue[pointID]+linkLength(301+i);
        updateNeighbourPointValue(paths[i][1]);
      }
    }
    if (startCdn[0] == linkCdn2[0] && startCdn[1] == linkCdn2[1]) {
      if (pointValue[paths[i][0]]>pointValue[pointID]+linkLength(301+i)) {
        pointValue[paths[i][0]] = pointValue[pointID]+linkLength(301+i);
        updateNeighbourPointValue(paths[i][0]);
      }
    }
  }
}
int[] pointID2cdn(int pID) {
  int cdn[] = {0, 0};
  switch(floor(pID/100)) {
  case 1:
    int id1 = pID-101;
    cdn[0] = hallPoints[id1][0];
    cdn[1] = hallPoints[id1][1];
    break;
  case 2:
    int id2 = pID-201;
    cdn[0] = centerPoints[id2][0];
    cdn[1] = centerPoints[id2][1];
    break;
  default:
    break;
  }
  return cdn;
}
int linkLength(int linkid) {
  linkid-=301;
  int pt1[] = pointID2cdn(paths[linkid][0]);
  int pt2[] = pointID2cdn(paths[linkid][1]);
  return round(dist(pt1[0], pt1[1], pt2[0], pt2[1]));
}

void drawRoute() {
  for (int i=0; i< route.length; i++) {
    if (route[i]==startPointID||route[i]==0)continue;
    int routeCdn[] = pointID2cdn(route[i]);
    int dir = round(degrees(atan2(routeCdn[1]-pointID2cdn(route[i-1])[1], routeCdn[0]-pointID2cdn(route[i-1])[0])));
    fill(0, 0, 255);
    textSize(25);
    String turn = "E";
    switch(lastDir-dir) {
    case 90:
    case -270:
      turn = "L";
      break;
    case 180:
    case -180:
      turn = "B";
      break;
    case 0:
      turn = "F";
      break;
    case -90:
    case 270:
      turn = "R";
      break;
    }
    stroke(0, 255, 255);
    line(pointID2cdn(route[i-1])[0], pointID2cdn(route[i-1])[1], routeCdn[0], routeCdn[1]);
    command[i-1] = turn + str(dist(pointID2cdn(route[i-1])[0], pointID2cdn(route[i-1])[1], routeCdn[0], routeCdn[1]));
    lastDir = dir;
    drawTriangle((pointID2cdn(route[i-1])[0]+routeCdn[0])/2, (pointID2cdn(route[i-1])[1]+routeCdn[1])/2, 15, dir);
    //text(command[i-1], pointID2cdn(route[i-1])[0], pointID2cdn(route[i-1])[1]);
    if (route[i]==endPointID) continue;
    fill(0, 255, 255);
    ellipse(routeCdn[0], routeCdn[1], 15, 15);
  }
}
String getCommand() {
  String cmdStr = "";
  for (int i=0; i<command.length; i++) {
    if (command[i]==null) break;
    cmdStr += command[i].substring(0, 1);// + '\t';
  }
  return cmdStr;
}
void drawTriangle(int centerX, int centerY, int radius, int direction) {
  triangle(centerX+radius*cos(radians(direction)), centerY+radius*sin(radians(direction)), 
    centerX+radius*cos(radians(direction+120)), centerY+radius*sin(radians(direction+120)), 
    centerX+radius*cos(radians(direction+240)), centerY+radius*sin(radians(direction+240)));
}