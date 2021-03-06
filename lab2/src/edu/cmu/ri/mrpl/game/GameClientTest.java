package edu.cmu.ri.mrpl.game;

import edu.cmu.ri.mrpl.kinematics2D.*;
import edu.cmu.ri.mrpl.maze.*;


public class GameClientTest {

    public static void main(String[] args)
    {
	GameClient gc = new GameClient();
	if(args.length != 3){
		System.out.println("Usage - java GameClientTest teamName name1 name2");
		System.exit(-1);
	}
	//set to some value, used in random loop
	int i = 0;
	//robots on your team
	boolean connected =gc.connectToServer(args[0],args[1],args[2]);
//	RealPose2D p = new RealPose2D(0.0,0.0,0.0);
//	RealPose2D goalPose = new RealPose2D(0.0,0.0,1.0);
	if (!connected)
	    System.out.println("Not actually connected, but still calling update functions");

	RealPose2D pose = new RealPose2D();
	RealPose2D goal = new RealPose2D();

	double t = 0;
	while (true) {
		gc.setPose(pose);
		gc.setGoal(goal);
		try {
		    Thread.sleep(100);
		} catch (Exception e) {}
		pose.setPose(4+2.5*Math.sin(Math.PI*(1+0.1*i)*t), 2+1.5*i-0.5*Math.cos(Math.PI*(1-2.5*i)*t), (i*0.9+0.3-3*t)*Math.PI);
		goal.setPose((3*Math.sin(Math.PI*2*t)+3)*i, (3*Math.sin(Math.PI*2*t)+3)*(1-i), 0);
		t += 0.01;

		if (i == 0) {
			if (((int)(t*10))%2 == 0)
				gc.fillDrop(0,0,MazeWorld.Direction.West);
			else
				gc.emptyDrop(0,0,MazeWorld.Direction.West);
			if (((int)(t*10))%3 == 1)
				gc.addGold(3,6,MazeWorld.Direction.East);
			else
				gc.removeGold(3,6,MazeWorld.Direction.East);
		}
		else {
			if (((int)(t*10))%2 == 1)
				gc.addWall(4,5,MazeWorld.Direction.West);
			else
				gc.removeWall(4,5,MazeWorld.Direction.West);
		}

		gc.addWall(-1, -1, MazeWorld.Direction.West);
	}

/*
	//if (connected)
	    {
		for (int i=0;i<20;i++)
		    {
			try {
			    Thread.sleep(1000);
			} catch (Exception e) {}
			gc.setGoal(goalPose);
			gc.setPose(p);
			gc.addWall(1,1,MazeWorld.Direction.West);
			gc.removeWall(2,2,MazeWorld.Direction.East);
			//gc.fillDrop(0,0,MazeWorld.Direction.West);
			//gc.emptyDrop(0,1,MazeWorld.Direction.East);
			
			gc.addGold(1,1,MazeWorld.Direction.East);
			p.setPose(p.getX()+0.1,p.getY(),p.getTh());
			System.out.println("SentPose\n");
		    }
	    }
*/	
	    

    }
}

