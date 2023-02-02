package frc.robot.subsystems;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

public class Dashboard{

    boolean[][] nodes;
    boolean selected;
    int j;
    int i;

    public Dashboard(){

        j=1;
        i=1;
        nodes = new boolean[3][3];
        selected = nodes[i][j];

        pushNode();
    }


    public void pushNode(){
       SmartDashboard.putBoolean("Node1", false);
       SmartDashboard.putBoolean("Node2", false);
       SmartDashboard.putBoolean("Node3", false);
       SmartDashboard.putBoolean("Node4", false);
       SmartDashboard.putBoolean("Node5", false);
       SmartDashboard.putBoolean("Node6", false);
       SmartDashboard.putBoolean("Node7", false);
       SmartDashboard.putBoolean("Node8", false);
       SmartDashboard.putBoolean("Node9", false);
    }

    public void selection(){
        
    }

}