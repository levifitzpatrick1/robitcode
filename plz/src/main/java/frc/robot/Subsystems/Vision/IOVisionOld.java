// package frc.robot.Subsystems.Vision;

// import org.littletonrobotics.junction.LogTable;
// import org.littletonrobotics.junction.inputs.LoggableInputs;



// public interface IOVisionOld  {

//     public static class IOVisionInputs implements LoggableInputs {
//         public double[] timestamps = new double[] {};
//         public double[][] frames = new double[][] {};
//         public double[] demoFrame = new double[] {};
//         public long fps = 0;

//         @Override
//         public void toLog(LogTable table) {
//             table.put("Timestamps", timestamps);
//             table.put("FrameCount", frames.length);
//             for (int i = 0; i < frames.length; i++) { 
//                 table.put("Frame/" + Integer.toString(i), frames[i]);
//             }

//             table.put("DemoFrame", demoFrame);
//             table.put("FPS",  fps);
//         }

//         @Override
//         public void fromLog(LogTable table) {
//             timestamps = table.getDoubleArray("Timestamps", timestamps);
//             int frameCount = (int) table.getInteger("FrameCount", 0);
//             frames = new double[frameCount][];

//             for (int i = 0; i < frameCount; i++) {
//                 frames[i] = table.getDoubleArray("Frame/" + Integer.toString(i), new double[] {});
//             }
//             demoFrame = table.getDoubleArray("DemoFrame", demoFrame);
//             fps = table.getInteger("FPS", fps);
//         }
//     }

//     public default void updateInputs(IOVisionInputs inputs) {}

// }
