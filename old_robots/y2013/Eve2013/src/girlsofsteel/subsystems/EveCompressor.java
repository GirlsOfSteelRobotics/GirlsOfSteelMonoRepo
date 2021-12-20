package girlsofsteel.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Subsystem;
import girlsofsteel.RobotMap;

public class EveCompressor extends Subsystem {



    //Changed so that compressor turns on/off automatically
    //Add back to smartdashboard for manual
//    private Relay compressorSpike;
    private final Compressor m_autoCompressor;

    public EveCompressor(){
//        compressorSpike = new Relay(RobotMap.COMPRESSOR_RELAY_PORT);
        m_autoCompressor = new Compressor(RobotMap.PRESSURE_SWITCH_CHANNEL);
        m_autoCompressor.start();
    }

//    public void runSpike(){
//        compressorSpike.set(Relay.Value.kForward);
//    }
//
//    public void stopSpike(){
//        compressorSpike.set(Relay.Value.kOff);
//    }

    @Override
    protected void initDefaultCommand() {
    }

}
