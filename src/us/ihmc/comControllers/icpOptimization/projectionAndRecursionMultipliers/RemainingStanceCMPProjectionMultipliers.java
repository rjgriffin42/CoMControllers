package us.ihmc.comControllers.icpOptimization.projectionAndRecursionMultipliers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import java.util.ArrayList;

public class RemainingStanceCMPProjectionMultipliers
{
   private static final String entryName = "StanceCMPEntryProjectionMultiplier";
   private static final String exitName = "StanceCMPExitProjectionMultiplier";
   private static final String previousExitName = "PreviousStanceCMPExitProjectionMultiplier";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable exitMultiplier;
   private final DoubleYoVariable entryMultiplier;
   private final DoubleYoVariable previousExitMultiplier;

   private final DoubleYoVariable omega;
   private final DoubleYoVariable doubleSupportSplitFraction;

   public RemainingStanceCMPProjectionMultipliers(String namePrefix, DoubleYoVariable omega, DoubleYoVariable doubleSupportSplitFraction,
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

   public void compute(double timeRemaining, ArrayList<DoubleYoVariable> doubleSupportDurations, boolean useTwoCMPs, boolean isInTransfer, boolean isInTransferEntry)
   {
      double timeSpentOnEndDoubleSupportCurrent = (1.0 - doubleSupportSplitFraction.getDoubleValue()) * doubleSupportDurations.get(0).getDoubleValue();

      double remainingProjection = Math.exp(omega.getDoubleValue() * timeRemaining);
      double endOfSupportProjection = Math.exp(omega.getDoubleValue() * timeSpentOnEndDoubleSupportCurrent);

      if (isInTransfer)
      {
         if (isInTransferEntry)
         {
            if (useTwoCMPs)
            {
               exitMultiplier.set(0.0);
               entryMultiplier.set(endOfSupportProjection - 1.0);
            }
            else
            {
               exitMultiplier.set(endOfSupportProjection - 1.0);
               entryMultiplier.set(0.0);
            }
            previousExitMultiplier.set(remainingProjection - endOfSupportProjection);
         }
         else
         {
            if (useTwoCMPs)
            {
               exitMultiplier.set(0.0);
               entryMultiplier.set(remainingProjection - 1.0);
            }
            else
            {
               exitMultiplier.set(remainingProjection - 1.0);
               entryMultiplier.set(0.0);
            }
            previousExitMultiplier.set(0.0);
         }
      }
      else
      {
         exitMultiplier.set(remainingProjection - 1.0);
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
