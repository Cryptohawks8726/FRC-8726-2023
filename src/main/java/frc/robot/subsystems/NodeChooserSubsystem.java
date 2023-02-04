package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import java.lang.Math;

//Remove Xbox Controller before Merging

public class NodeChooserSubsystem extends SubsystemBase {

    XboxController remote1;
    boolean[][] nodes;
    //boolean selected;
    int j;
    int i;
    int delay;
    int previousI;
    int previousJ;

    public NodeChooserSubsystem(){

        remote1 = new XboxController(0);
        nodes = new boolean[3][3];
        j=1;
        i=1;
        previousI = i;
        previousJ = j;
        delay = 0;
        //selected = nodes[i][j];
      
        for (int n = 0; n < 3; n++){
            for (int p = 0; p < 3; p++){
                nodes[n][p] = false;
            }
        }
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

        if (delay > 0){
            delay--;
        }

        int pov = remote1.getPOV();
        System.out.println(pov);

        if((pov == 0 || pov == 90 || pov == 180 || pov == 270) && delay == 0){

            nodes[i][j] = false;

            switch (pov){
                case 0:
                    i--;
                    break;
                case 90:
                    j++;
                    break;
                case 180:
                    i++;
                    break;
                case 270:
                    j--;
                    break;
                default:
            }
        
            i = Math.max(0, Math.min(2, i));
            j = Math.max(0, Math.min(2, j));
            
            if (i != previousI || j != previousJ){
                previousI = i;
                previousJ = j;
                delay = 15;
            }

            nodes[i][j] = true; 
        }
    }

    @Override
    public void periodic(){
        
        selection(); 
        pushNode();

    }

    public int[] returnPoint(){
    
     for(int i = 0; i < 3; i++){

        for(int j = 0; j < 3; j++){

            int[] trueNodes = {i, j};

            if (nodes[i][j] == true){

                return trueNodes;

            }
        }
     }   

     return null;

    }

}