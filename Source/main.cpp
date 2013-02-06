//
//  main.cpp
//  Millipede
//
//  Created by Jingyi Fang on 2/10/11.
//  Copyright 2011 Jingyi Fang. All rights reserved.
//

#include "main.h"

void initScene(){

	std::cout<<"Initiallizing The System...."<<std::endl;

	std::cout<<"Setting up Light, Camera and Clock...."<<std::endl;
    //set up the camera
    Pentax.Init(Eigen::Vector4f(0,250,250.0,1.0),Eigen::Vector4f(0,0,0.0,1.0), Eigen::Vector4f(0,1.0,0,0), 
		60, Window_Width/Window_Height , 1.0, 1000);
    
    //set up the light
    Lumia.m_position = Pentax.m_position;
    Lumia.m_color = Eigen::Vector4f(1.0,1.0,1.0,1.0);//white light

	//set up the world
	myWorld = new World(50000);

	//set up the drawer
	myDrawer = new Drawer;
	myDrawer->PushMatrix();

    std::cout<<"Setting up the World..."<<std::endl;

	myTerrain = new Terrain(Eigen::Vector2f(100,100), Eigen::Vector2i(100,100), 10, TERRAIN_RANDOM);

	reinitScene();

	std::cout<<"Starting Animation..."<<std::endl;


}

void reinitScene(){

	//press SpaceBar to trigger

	myWorld->Clear();//clear everything
	myWorld->Add_Object(myTerrain);//add back the terrain

	Eigen::Vector3i deform_res(5,5,5);double youngs_modulus = 2000;
	Eigen::Vector3f temp_position;Eigen::Vector3f rigid_size(2,2,2); double deform_length = 2;
	std::vector<Node*> temp_nodes;

	//create the first rigid skeleton 
	RigidCube* cube1 = new RigidCube;
	cube1->Init(1.0, Eigen::Vector3f(0,10,0), rigid_size, Eigen::Vector3f(1,0,0));
	cube1->m_fixed = true;

	//create the first deformable section;
	Deformable3D* deform1 = new Deformable3D;
	temp_position = cube1->m_Center 
		+ Eigen::Vector3f(0.5*rigid_size[0], -0.5*rigid_size[1], -0.5*rigid_size[2]);
	deform1->Init(deform_res,1.0,youngs_modulus,0.4,100.0,temp_position,
			Eigen::Vector3f(deform_length,rigid_size[1],rigid_size[2]),Eigen::Vector3f(1,1,1));

	//attach the previous rigid part and deformable part
	temp_nodes = deform1->m_Mesh->GetLeftNodes();
	cube1->AttachNodes(temp_nodes);

	//create the second rigid skeleton 
	RigidCube* cube2 = new RigidCube;
	cube2->Init(1.0, Eigen::Vector3f(cube1->m_Center[0] + deform_length + rigid_size[0],10, 0), rigid_size, Eigen::Vector3f(1,0,0));
	cube2->m_fixed = false;

	//attach the previous deformable part with the rigid part
	temp_nodes = deform1->m_Mesh->GetRightNodes();
	cube2->AttachNodes(temp_nodes);

	//create the second deformable section;
	Deformable3D* deform2 = new Deformable3D;
	temp_position = cube2->m_Center 
		+ Eigen::Vector3f(0.5*rigid_size[0], -0.5*rigid_size[1], -0.5*rigid_size[2]);
	deform2->Init(deform_res,1.0,youngs_modulus,0.4,100.0,temp_position,
			Eigen::Vector3f(deform_length,rigid_size[1],rigid_size[2]),Eigen::Vector3f(1,1,1));

	//attach the previous rigid part and deformable part
	temp_nodes = deform2->m_Mesh->GetLeftNodes();
	cube2->AttachNodes(temp_nodes);

	//create the first rigid skeleton 
	RigidCube* cube3 = new RigidCube;
	cube3->Init(1.0, Eigen::Vector3f(cube2->m_Center[0] + deform_length + rigid_size[0],10, 0), rigid_size, Eigen::Vector3f(1,0,0));
	cube3->m_fixed = false;

	//attach the previous deformable part with the rigid part
	temp_nodes = deform2->m_Mesh->GetRightNodes();
	cube3->AttachNodes(temp_nodes);

	myWorld->Add_Object(cube1);
	myWorld->Add_Object(deform1);
	myWorld->Add_Object(cube2);
	myWorld->Add_Object(deform2);
	myWorld->Add_Object(cube3);


	TIME_LAST = TM.GetElapsedTime() ;
	DTIME = 0.0;
	FRAME_TIME = 0.0;


}

void drawScene(){
    
    glEnable( GL_DEPTH_TEST );
    glClearColor(0.0, 0.0, 0.0, 0.0);//Black background

	Pentax.Update(DTIME);

    Lumia.m_position = Pentax.m_position;//the light is attached to the camera
	
	myWorld->Draw(DRAW_TYPE, Pentax, Lumia);

}

void keyboardCallback(unsigned char key, int x, int y){

	if(CONTROL == 1){
		switch(key)
		{
			case '7'://down
			break;
		}
	}
    
	if ( key == EscKey || key == 'q' || key == 'Q' ) 
    {
        exit(0);
    }
    if( key == 's'|| key == 'S')
    {
        STOP *= -1; 
    }
	if( key == 'p'|| key == 'P' )
	{
		PICK *= -1;
		if(PICK == 1)
			std::cout<<"Picking Mode"<<std::endl;
		if(PICK == -1)
			std::cout<<"Rotating Mode"<<std::endl;
	}
    
    if( key == '1')
        DRAW_TYPE = DRAW_MESH;
    if( key == '2')
        DRAW_TYPE = DRAW_PHONG;
    if( key == '3')
        DRAW_TYPE = DRAW_TEXTURE;

        
    //reset the scene and camera
    if ( key == SpaceKey) {
        reinitScene();
        glutSwapBuffers();
    }
}

void displayCallback(){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawScene();
    glutSwapBuffers();

}

void reshapeCallback(int w, int h){
    Window_Width = w;
    Window_Height = h;

    glViewport(0, 0, w, h);
	Pentax.m_aspect = (float)w/(float)h;
	glutPostRedisplay() ;

}

void motionCallback(int x, int y){
     
    if( Button == GLUT_LEFT_BUTTON )
    {
		CursorX = double(2*x-Window_Width)/(Window_Width);
		CursorY = double(Window_Height-2*y)/(Window_Height);
        
		Pentax.MouseDrag(CursorX, CursorY);
		
        glutPostRedisplay() ;
    }
    else if( Button == GLUT_RIGHT_BUTTON )
    {
        if( y - PrevY > 0 )
            Pentax.m_zoom  = Pentax.m_zoom  * 1.03 ;
        else 
            Pentax.m_zoom   = Pentax.m_zoom  * 0.97 ;
        PrevY = y ;
        glutPostRedisplay() ;
    }
}

void mouseCallback(int button, int state, int x, int y){

	CursorX = double(2*x-Window_Width)/(Window_Width);
	CursorY = double(Window_Height-2*y)/(Window_Height);

	Button = button;
	if( Button == GLUT_LEFT_BUTTON && state == GLUT_DOWN )
	{
		if(PICK == -1){
			Pentax.MouseLeftDown(CursorX,CursorY);
		}

	}
	if( Button == GLUT_LEFT_BUTTON && state == GLUT_UP )
	{
		if(PICK == -1){
			Pentax.MouseLeftUp(CursorX,CursorY);
			Button = -1 ;
		}
 
		
	}
	if( button == GLUT_RIGHT_BUTTON && state == GLUT_UP )
	{
		if(PICK == 1){
			//World.Pick(x,y);
		}
	}
    
    if( Button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN )
    {
        PrevY = y ;
    }
	glutPostRedisplay() ;
}

void cursorCallback(int x, int y){

	CursorX = double(2*x-Window_Width)/(Window_Width);
	CursorY = double(Window_Height-2*y)/(Window_Height);
}

void idleCallback(){
    
	TIME = TM.GetElapsedTime() ;
        
	DTIME = TIME - TIME_LAST;
	TIME_LAST = TIME;

	if(DTIME > 1/1000.0)
		DTIME = 1/1000.0;
	
	DTIME = 1/2000.0;//fixed dt

	FRAME_TIME += DTIME;

	if(STOP == -1){
	//only update physics
		myWorld->Update(DTIME);
	}

	if(FRAME_TIME > 0.01)//33 frames per second
	{
		glutPostRedisplay() ; //draw new frame
		FRAME_TIME = 0;	
		FRAME_COUNT++;
		//OUTPUT_ONE_FRAME();
	}
	


	//printf("Physics Rate %f\r", 1.0/DTIME) ;
}


int main (int argc, char ** argv){
    // init GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);

	glutInitWindowSize(Window_Width,Window_Height);
    glutCreateWindow("Rigid-Deformable Coupling - Franklin Fang");
    // init GLUT callbacks
    glutIdleFunc(idleCallback);
	glutReshapeFunc (reshapeCallback);
    glutKeyboardFunc(keyboardCallback);
    glutMouseFunc(mouseCallback) ;
    glutMotionFunc(motionCallback);
	glutPassiveMotionFunc(cursorCallback);
    glutDisplayFunc(displayCallback);
	glewInit();
    initScene();
    glutMainLoop();
    
    return 0;
}

