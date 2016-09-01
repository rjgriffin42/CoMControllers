package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.discontinuous;

import us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers.RemainingStanceCMPProjectionMultipliers;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class DiscontinuousRemainingStanceCMPProjectionMultipliers implements RemainingStanceCMPProjectionMultipliers
{
   private static final String entryName = "StanceCMPEntryRemainingProjectionMultiplier";
   private static final String exitName = "StanceCMPExitRemainingProjectionMultiplier";
   private static final String previousExitName = "PreviousStanceCMPExitRemainingProjectionMultiplier";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable exitMultiplier;
   private final DoubleYoVariable entryMultiplier;
   private final DoubleYoVariable previousExitMultiplier;

   private final DoubleYoVariable omega;
   private final DoubleYoVariable doubleSupportSplitFraction;

   public DiscontinuousRemainingStanceCMPProjectionMultipliers(String namePrefix, DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitFraction,
         YoVariableRegistry parentRegistry)
   {
      this.omega = omega;
      this.doubleSupportSplitFraction = doubleSupportSplitFraction;

      exitMultiplier = new DoubleYoVariable(namePrefix + exitName, registry);
      entryMultiplier = new DoubleYoVariable(namePrefix + entryName, registry);
      previousExitMultiplier = new DoubleYoVariable(namePrefix + previousExitName, registry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      exitMultiplier.set(0.0);
      entryMultiplier.set(0.0);
      previousExitMultiplier.set(0.0);
   }

   public void compute(double timeRemaining, ArrayList<DoubleYoVariable> doubleSupportDurations, ArrayList<DoubleYoVariable> singleSupportDurations,
         boolean useTwoCMPs, boolean isInTransfer, boolean isInTransferEntry)
   {
      double timeSpentOnEndDoubleSupportCurrent = (1.0 - doubleSupportSplitFraction.getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue();

      double remainingProjection = Math.exp(-omega.getDoubleValue() * timeRemaining);
      double endOfSupportProjection = Math.exp(omega.getDoubleValue() * (timeSpentOnEndDoubleSupportCurrent - timeRemaining));

      if (isInTransfer)
      {
         if (isInTransferEntry)
         {
            if (useTwoCMPs)
            {
               exitMultiplier.set(0.0);
               entryMultiplier.set(endOfSupportProjection - remainingProjection);
            }
            else
            {
               exitMultiplier.set(endOfSupportProjection - remainingProjection);
               entryMultiplier.set(0.0);
            }
            previousExitMultiplier.set(1.0 - endOfSupportProjection);
         }
         else
         {
            if (useTwoCMPs)
            {
               exitMultiplier.set(0.0);
               entryMultiplier.set(1.0 - remainingProjection);
            }
            else
            {
               exitMultiplier.set(1.0 - remainingProjection);
               entryMultiplier.set(0.0);
            }
            previousExitMultiplier.set(0.0);
         }
      }
      else
      {
         exitMultiplier.set(1.0 - remainingProjection);
         entryMultiplier.set(0.0);
         previousExitMultiplier.set(0.0);
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

   public double getPreviousExitMultiplier()
   {
      return previousExitMultiplier.getDoubleValue();
   }
}
