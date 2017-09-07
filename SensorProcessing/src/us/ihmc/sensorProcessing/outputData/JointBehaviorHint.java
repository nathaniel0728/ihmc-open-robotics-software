package us.ihmc.sensorProcessing.outputData;

public enum JointBehaviorHint
{

   // The following 3 modes are considered "main" modes
   NONE(null), /** No special hints for joint behavior **/
   HOLDING_POSITION(NONE), /** The high level controller is trying to hold a constant position. **/
   DYNAMIC_TORQUE_CONTROL(NONE), /** The high level controller is using dynamic torque control. **/
   
   
   // These are all optional modes and fall back to a main mode
   DYNAMIC_TRACKING_POSITION(HOLDING_POSITION), /** The controller is trying to track fast trajectories **/
   STAND_PREP(HOLDING_POSITION), /** The controller is in stand prep. **/
   
   ;
   
   private final JointBehaviorHint fallback;
   JointBehaviorHint(JointBehaviorHint fallback)
   {
      this.fallback = fallback;
   }
   
   
   /**
    * This function returns the current hint, or falls back to a implemented hint.
    * 
    * 
    * @param implementedHints
    * @throws RuntimeException if NONE is not in the list of implemented hints.
    * 
    * @return This hint or a suitable fallback
    */
   public JointBehaviorHint getHintOrFallback(JointBehaviorHint... implementedHints)
   {
      for(int i = 0; i < implementedHints.length; i++)
      {
         if(implementedHints[i] == this)
         {
            return implementedHints[i];
         }
      }
      
      if(fallback != null)
      {
         return getHintOrFallback(implementedHints);
      }
      
      throw new RuntimeException(this.toString() + " does not have a fallback hint and is not implemented");
   }
}
