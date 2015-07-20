/*Written by Andrew Barrette
without OOP to allow future application on microcontroller*/

//g++ -o MLtestC#.# MLtestC#.#.cpp glut32.lib -lopengl32 -lglu32 -lopenal32 -static-libgcc -static-libstdc++  //use this format to compile
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <windows.h>
#include "GL/glut.h"
#include <cmath>
#include <vector>
using namespace std;

#define BUILDID "1.20"
/*Changes:
    - Upgraded to Advantage Learning.
    - Improved collision detection algorithms. Bot no longer passes through walls.
    - Added an auxillary chooseaction algorithm, in case it is desired in the future.
    - "intersection" points can now be graphically displayed (for debugging purposes).
*/

#define TWOPI 6.28319

//Simulation parameters
#define STATENUM pow(3,RAYNUM)*2
#define ACTIONNUM 4

#define ROOMVERTICES 10
#define VELOCITY .2*BOTSIZE
#define ANGVELOCITY 2*VELOCITY
#define SIGHTRANGE 5*BOTSIZE
#define PADSIZE BOTSIZE/2.0
#define BOTSIZE .2
#define SIGHTFOV (TWOPI/4)
#define RAYNUM 5
#define STANDARDREWARD 1
#define REWARDBIAS (-STANDARDREWARD/100.0)
#define AINIT 0
#define GAMMA .8
#define TIMEPERUPDATE 1 //must be a natural number
#define TEMPERATURE .1
#define P_USEMAX 0.9

#define STATESTR "ray1 r\tg\tray2 r\tg\tray3 r\tb\tsensor"
#define ACTIONSTR "left wheel\tright wheel"


GLint winw=400,winh=400;
GLfloat viewradius=2.5,pbot[3]={-.8,.3,0},pcharger[2]={1,0},room[10][2];
int t=0,logtimestamp,state,action,newstate;
float A[243*9*2][ACTIONNUM],reward,alpha=.1,walldistance;
bool dispbit=true,pausebit=false,chargergrabbed=false,stuck=false;

float randnorm(float SD){
    return SD*sqrt(-2*log(((rand()%10000)+1)/10001.0))*cos(TWOPI*(rand()%10000)/10000.0);
}

bool checkline(float* p0,float* p1,float* p2,float* intersection){
    //If triangle formed by points has an obtuse angle, then point is outside of the ends of the line segment
    if(((p1[0]-p0[0])*(p1[0]-p2[0])+(p1[1]-p0[1])*(p1[1]-p2[1]))*((p2[0]-p0[0])*(p2[0]-p1[0])+(p2[1]-p0[1])*(p2[1]-p1[1]))<0)return false;
    //Otherwise, return the point on the line formed by p1p2 that is closest to p0
    float m=(p2[1]-p1[1])/(p2[0]-p1[0]),b=p1[1]-m*p1[0];
    intersection[0]=(p0[0]+m*p0[1]-m*b)/(m*m+1);
    intersection[1]=(m*(p0[0]+m*p0[1])+b)/(m*m+1);
    return true;
}
bool checkstuck(float* p){
    int i;
    float intersection[2];
    //if(dispbit || t%200==0)glClear(GL_COLOR_BUFFER_BIT);		     // Clear Screen and Depth Buffer
    
    //Check that every wall forms a counter-clockwise triangle with the point.
    for(i=0;i<ROOMVERTICES;i++){
        if(checkline(p,room[i],room[(i+1)%ROOMVERTICES],intersection)){
            
            //Display wall intersections where applicable.
            /*if(dispbit || t%200==0){
                glViewport(0,0,winw,winh);
                glMatrixMode(GL_PROJECTION);
                glLoadIdentity();
                gluOrtho2D(-viewradius*winw/winh,viewradius*winw/winh,viewradius,-viewradius);
                
                glPointSize(6);
                glColor3f(0,0,1);
                glBegin(GL_POINTS);
                    glVertex2fv(intersection);
                glEnd();
            }*/
            
            if((room[i][0]-p[0])*(room[i][1]+p[1])+(room[i+1][0]-room[i][0])*(room[i+1][1]+room[i][1])+(p[0]-room[i+1][0])*(p[1]+room[i+1][1])>0)return true;
            else if(sqrt(pow(p[0]-intersection[0],2)+pow(p[1]-intersection[1],2))<=BOTSIZE*sqrt(2))return true;
        }
    }
    return false;
}

int stateID(){
    int i,wall,inputval[RAYNUM+1];
    float ray=0,r,j,distance,intersection[2];
    inputval[RAYNUM]=0;
    //Cycle through visual rays
    for(i=0,ray=-1;i<RAYNUM,ray<=1;i++,ray+=2.0/(RAYNUM-1)){
        inputval[i]=0;
        //For each ray, step along the length
        for(r=BOTSIZE;r<SIGHTRANGE;r+=r*SIGHTFOV/((RAYNUM-1)*2.0)){
            //At each point along ray, check for intersection with charger
            float raypoint[2]={pbot[0]+r*cos(pbot[2]+ray*SIGHTFOV/2.0),pbot[1]+r*sin(pbot[2]+ray*SIGHTFOV/2.0)};
            distance=sqrt(pow(raypoint[0]-pcharger[0],2)+pow(raypoint[1]-pcharger[1],2));
            if(distance-PADSIZE<=r*SIGHTFOV/((RAYNUM-1)*2.0)){
                inputval[i]=2;
                distance=sqrt(pow(pbot[0]-pcharger[0],2)+pow(pbot[1]-pcharger[1],2));
                if(distance<BOTSIZE+PADSIZE){
                    inputval[RAYNUM]=1;
                    if(ray==0)reward+=STANDARDREWARD;
                }
                break;
            }
            //At each point along ray, check for intersection with wall
            for(wall=0;wall<ROOMVERTICES;wall++){
                if(!checkline(raypoint,room[wall],room[(wall+1)%ROOMVERTICES],intersection))continue;
                distance=sqrt(pow(raypoint[0]-intersection[0],2)+pow(raypoint[1]-intersection[1],2));
                if(distance<=r*SIGHTFOV/((RAYNUM-1)*2.0)){
                    inputval[i]=1;
                    break;
                }
            }
            if(inputval[i]>0)break;
        }
    }
    
    //Check for wall touch
    if(inputval[RAYNUM]==0)if(checkstuck(pbot))inputval[RAYNUM]=1;
    
    //Convert set of input integers to a single integer representing a unique state
    int newstate=0,multiplier[RAYNUM+1];
    multiplier[0]=1;
    for(i=1;i<RAYNUM;i++)multiplier[i]=3;
    multiplier[RAYNUM]=2;
    for(i=0;i<RAYNUM+1;i++){
        if(i>0)multiplier[i]*=multiplier[i-1];
        newstate+=inputval[i]*multiplier[i];
    }
    return newstate;
}

int chooseaction(){
    reward=REWARDBIAS;
    state=stateID();
    
    //Choose the most valuable action with some probably. If not, discard action and repeat.
    /*int quantity=ACTIONNUM,actionlist[ACTIONNUM],i,j,maxi,temp;
    for(i=0;i<quantity;i++)actionlist[i]=i;
    while(quantity>1){
        maxi=0;
        for(i=1;i<quantity;i++)
            if(A[state][actionlist[i]]>A[state][actionlist[maxi]])maxi=i;
        if((rand()%10000)/10000.0<P_USEMAX)return maxi;
        else{
            i=j=0;
            while(i<quantity){
                actionlist[j]=actionlist[i];
                if(i==maxi)j--;
                i++;
                j++;
            }
            quantity--;
        }
    }
    return maxi;*/
    
    //Use temperature to choose an action
    int i;
    float sum=0,r=(rand()%10000)/10000.0;
    for(i=0;i<ACTIONNUM;i++)sum+=exp(A[state][i]/TEMPERATURE);
    for(i=0;i<ACTIONNUM;i++){
        if(r<exp(A[state][i]/TEMPERATURE)/sum)return i;
        else r-=exp(A[state][i]/TEMPERATURE)/sum;
    }
    return 0;
}

void updateA(float oldreward,int newstate){
    int i,maxi1=0,maxi2;
    float maxsofar=A[state][0];
    for(i=1;i<ACTIONNUM;i++)
        if(A[state][i]>maxsofar){
            maxsofar=A[state][i];
            maxi1=i;
        }
    maxsofar=A[newstate][0];
    for(i=1;i<ACTIONNUM;i++)
        if(A[newstate][i]>maxsofar){
            maxsofar=A[newstate][i];
            maxi2=i;
        }            
    A[state][action]=(1-alpha)*A[state][action]+(alpha/TIMEPERUPDATE)*(oldreward+pow(GAMMA,TIMEPERUPDATE)*A[newstate][maxi2]-A[state][maxi1])+alpha*A[state][maxi1];
    if(dispbit)cout<<"\nstuck="<<stuck<<",\trw="<<oldreward<<",\ts="<<state<<",\ta="<<action<<",\tA(s,a)="<<A[state][action];
    state=newstate;
}

void applyaction(){
    if(action>1){
        switch(action){
            case 2:
                pbot[2]+=ANGVELOCITY; 
                break;
            case 3:
                pbot[2]-=ANGVELOCITY;
                break;
        }
    }
    else{
        GLfloat newpbot[3];
        newpbot[0]=pbot[0];
        newpbot[1]=pbot[1];
        switch(action){
            case 0:
                newpbot[0]+=2*VELOCITY*cos(pbot[2]);
                newpbot[1]+=2*VELOCITY*sin(pbot[2]);
                break;
            case 1:
                newpbot[0]-=2*VELOCITY*cos(pbot[2]);
                newpbot[1]-=2*VELOCITY*sin(pbot[2]);
                break;
        }
        if(checkstuck(newpbot)){
            stuck=true;
            reward-=STANDARDREWARD;
        }
        else{
            stuck=false;
            pbot[0]=newpbot[0];
            pbot[1]=newpbot[1];
        }
    }
}
        
void disp(){
	if(dispbit || t%200==0){
        int i;
        float j;
        glClear(GL_COLOR_BUFFER_BIT);		     // Clear Screen and Depth Buffer
        
        //This is necessary for zoom updates.
    	glViewport(0,0,winw,winh);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluOrtho2D(-viewradius*winw/winh,viewradius*winw/winh,viewradius,-viewradius);
        
        //Disply room
            glColor3f(1,0,0);
            glBegin(GL_LINE_LOOP);
            	for(i=0;i<ROOMVERTICES;i++)glVertex2f(room[i][0],room[i][1]);
            glEnd();
        //Disply bot
            glColor3f(1,1,(float)(!stuck));
            glBegin(GL_QUADS);
                for(i=0;i<=4;i++)glVertex2f(pbot[0]+BOTSIZE*sqrt(2)*cos(pbot[2]+TWOPI*(0.125+0.25*i)),pbot[1]+BOTSIZE*sqrt(2)*sin(pbot[2]+TWOPI*(0.125+0.25*i)));
            glEnd();
            glColor3f(.5,.5,.5);
            //Display rays
            glBegin(GL_LINES);
                for(j=-1;j<=1;j+=2.0/(RAYNUM-1)){
                    glVertex2f(pbot[0],pbot[1]);
                    glVertex2f(pbot[0]+SIGHTRANGE*cos(pbot[2]+j*SIGHTFOV/2.0),pbot[1]+SIGHTRANGE*sin(pbot[2]+j*SIGHTFOV/2.0));
                }
            glEnd();
        //Display charger
            glColor3f(0,1,0);
            glBegin(GL_QUADS);
                glVertex2f(pcharger[0]+PADSIZE,pcharger[1]+PADSIZE);
                glVertex2f(pcharger[0]-PADSIZE,pcharger[1]+PADSIZE);
                glVertex2f(pcharger[0]-PADSIZE,pcharger[1]-PADSIZE);
                glVertex2f(pcharger[0]+PADSIZE,pcharger[1]-PADSIZE);
            glEnd();
        glutSwapBuffers();
    }
    
    if(!pausebit){
        if(t%TIMEPERUPDATE==0){
            action=chooseaction();
        }
        applyaction();
        if(t%TIMEPERUPDATE==0){
            updateA(reward,stateID());
        }
        t++;
    }
}

void resize(int x,int y){
    if(x>0)winw=x;
    if(y>0)winh=y;
	glViewport(0,0,winw,winh);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-viewradius*winw/winh,viewradius*winw/winh,viewradius,-viewradius);
}

void init() 
{
    srand(time(NULL));
	glViewport(0,0,winw,winh);
	glClearColor(0.0, 0.0, 0.0, 1.0);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-viewradius,viewradius,viewradius,-viewradius);
    logtimestamp=time(NULL);
    cout<<"\nSimulation timestamp = "<<logtimestamp;
    
    //Initialize room
    int i,j;
    for(i=0;i<ROOMVERTICES;i++){
        float r=2*(rand()%1000)/1000.0+1;
        room[i][0]=r*cos(TWOPI*i/(float)ROOMVERTICES);
        room[i][1]=r*sin(TWOPI*i/(float)ROOMVERTICES);
    }
    //Initialize Q
    for(i=0;i<STATENUM;i++)
        for(j=0;j<ACTIONNUM;j++)
            A[i][j]=AINIT+randnorm(AINIT/10.0);
}

void mouseclick(int button, int clickstate, int x, int y){   
    if(clickstate==GLUT_DOWN && abs(viewradius*((float)x*2.0/winw-1.0)-pcharger[0])<BOTSIZE/2.0 && abs(viewradius*((float)y*2.0/winh-1.0)-pcharger[1])<BOTSIZE/2.0)chargergrabbed=true;
    else if(clickstate==GLUT_UP)chargergrabbed=false;
}

void mousemotion(int x,int y){
    if(chargergrabbed){
        pcharger[0]=viewradius*(2.0*(float)x/winw-1);
        pcharger[1]=viewradius*(2.0*(float)y/winh-1);
    }
}

void keyfunc(unsigned char key,int x,int y){
    switch(key){
        case ' ': dispbit=!dispbit; break;
        case 'p': pausebit=!pausebit; break;
        case '=': viewradius/=1.2; break;
        case '-': viewradius*=1.2; break;
    }
}

int main(int argc, char **argv) 
{
	glutInit(&argc, argv);                                                      // GLUT initialization
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);                                // Display Mode
	glutInitWindowSize(winw,winh);					                            // set window size
	glutCreateWindow("Reinforcement Learning Simulation");							// create Window
	glutDisplayFunc(disp);									                    // register Display Function
	glutIdleFunc(disp);
	glutReshapeFunc(resize);
	glutKeyboardFunc(keyfunc);
	glutMouseFunc(mouseclick);
	glutMotionFunc(mousemotion);
	//glutSpecialFunc(specialkeyfunc);
	//glutKeyUpFunc(keyupfunc);
	init();
	glutMainLoop();												                // run GLUT mainloop
	return 0;
}
