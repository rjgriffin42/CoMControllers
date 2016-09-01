package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class StanceCMPProjectionMultipliers
{
   private static final String entryName = "StanceCMPEntryProjectionMultiplier";
   private static final String exitName = "StanceCMPExitProjectionMultiplier";
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable exitMultiplier;
   private final DoubleYoVariable entryMultiplier;

   private final BooleanYoVariable useTwoCMPs;
   private final BooleanYoVariable isInTransfer;

   private final DoubleYoVariable omega;
   private final DoubleYoVariable doubleSupportSplitFraction;
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime;

   public StanceCMPProjectionMultipliers(String namePrefix, DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitFraction,
         DoubleYoVariable exitCMPDurationInPercentOfStepTime, BooleanYoVariable useTwoCMPs, BooleanYoVariable isInTransfer, YoVariableRegistry parentRegistry)
   {
      this.omega = omega;
      this.doubleSupportSplitFraction = doubleSupportSplitFraction;
      this.exitCMPDurationInPercentOfStepTime = exitCMPDurationInPercentOfStepTime;
      this.useTwoCMPs = useTwoCMPs;
      this.isInTransfer = isInTransfer;

      exitMultiplier = new DoubleYoVariable(namePrefix + exitName, registry);
      entryMultiplier = new DoubleYoVariable(namePrefix + entryName, registry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      exitMultiplier.set(0.0);
      entryMultiplier.set(0.0);
   }

   public void compute(ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations)
   {
      double firstStepTime = doubleSupportDurations.get(0).getDoubleValue() + singleSupportDurations.get(0).getDoubleValue();
      double timeSpentOnInitialDoubleSupportUpcoming = doubleSupportSplitFraction.getDoubleValue() * doubleSupportDurations.get(1).getDoubleValue();
      double timeSpentOnEndDoubleSupportCurrent = (1.0 - doubleSupportSplitFraction.getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue();

      if (useTwoCMPs.getBooleanValue())
      {

         if (isInTransfer.getBooleanValue())
         {
            exitMultiplier.set(1.0 - Math.exp(-omega.getDoubleValue() * timeSpentOnInitialDoubleSupportUpcoming));
            entryMultiplier.set(0.0);
         }
         else
         {
            double totalTimeSpentOnExitCMP = exitCMPDurationInPercentOfStepTime.getDoubleValue() * firstStepTime;
            double totalTimeSpentOnEntryCMP = (1.0 - exitCMPDurationInPercentOfStepTime.getDoubleValue()) * firstStepTime;

            double projectionTime = totalTimeSpentOnEntryCMP + timeSpentOnEndDoubleSupportCurrent;
            double multiplier = Math.exp(-omega.getDoubleValue() * projectionTime);

            exitMultiplier.set(multiplier * (1.0 - Math.exp(-omega.getDoubleValue() * totalTimeSpentOnExitCMP)));
            entryMultiplier.set(1.0 - multiplier);
         }
      }
      else
      {
         double timeToFinish = timeSpentOnInitialDoubleSupportUpcoming;

         if (isInTransfer.getBooleanValue())
            timeToFinish += singleSupportDurations.get(0).getDoubleValue();

         exitMultiplier.set(1.0 - Math.exp(-omega.getDoubleValue() * timeToFinish));
         entryMultiplier.set(0.0);
      }
   }

   public double getExitMultiplier()
   {
      return exitMultiplier.getDoubleValue();
   }

   public double getEntryMultiplier()
   {
      return entryMultiplier.getDoubleValue();
   }
}
