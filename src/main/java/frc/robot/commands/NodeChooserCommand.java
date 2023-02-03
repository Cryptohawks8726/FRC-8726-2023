package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import java.lang.Math;
//Remove Xbox Controller before Merging

public class NodeChooserCommand extends CommandBase {

    XboxController remote1;
    boolean[][] nodes;
    boolean selected;
    int j;
    int i;

    public NodeChooserCommand(){

        j=1;
        i=1;
        nodes = new boolean[3][3];
        //selected = nodes[i][j];
        remote1 = new XboxController(0);
        
        
    }

    public void pushNode(){
       SmartDashboard.putBoolean("Node1", nodes[0][0]);
       SmartDashboard.putBoolean("Node2", nodes[0][1]);
       SmartDashboard.putBoolean("Node3", nodes[0][2]);
       SmartDashboard.putBoolean("Node4", nodes[1][0]);
       SmartDashboard.putBoolean("Node5", nodes[1][1]);
       SmartDashboard.putBoolean("Node6", nodes[1][2]);
       SmartDashboard.putBoolean("Node7", nodes[2][0]);
       SmartDashboard.putBoolean("Node8", nodes[2][1]);
       SmartDashboard.putBoolean("Node9", nodes[2][2]);
    }

    public void selection(){

        int pov = remote1.getPOV();
        System.out.println(pov);

            if(pov == 0 || pov == 90 || pov == 180 || pov == 270){

                nodes[i][j] = false;

                switch (pov){
                    case 0:
                        i++;
                        break;
                    case 90:
                        j++;
                        break;
                    case 180:
                        i--;
                        break;
                    case 270:
                        j--;
                        break;
                    default:
                }
            
                i = Math.max(0, Math.min(2, i));
                j = Math.max(0, Math.min(2, j));
                
                nodes[i][j] = true; 

            }
    }

    @Override
    public void execute(){
        selection();
        pushNode();

    }


}