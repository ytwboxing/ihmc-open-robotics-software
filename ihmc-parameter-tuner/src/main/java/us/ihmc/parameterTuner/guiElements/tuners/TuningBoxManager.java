package us.ihmc.parameterTuner.guiElements.tuners;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javafx.application.Platform;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.layout.VBox;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;

public class TuningBoxManager
{
   private final VBox tuningBox;
   private final List<String> parametersBeingTuned = new ArrayList<>();
   private final Map<String, Tuner> tunerMap = new HashMap<>();

   public TuningBoxManager(VBox tuningBox)
   {
      this.tuningBox = tuningBox;
   }

   public void handleNewParameter(GuiParameter parameter)
   {
      String parameterName = parameter.getName();
      if (parametersBeingTuned.contains(parameterName))
      {
         Platform.runLater(() -> {
            Alert alert = new Alert(AlertType.INFORMATION);
            alert.setTitle("Information Dialog");
            alert.setHeaderText("Can not add parameter");
            alert.setContentText("A parameter with the same name is being tuned.");
            alert.showAndWait();
         });
         return;
      }

      Tuner tuner = tunerMap.get(parameter.getUniqueName());
      parametersBeingTuned.add(parameterName);
      EventHandler<ActionEvent> closeHandler = event -> {
         tuningBox.getChildren().remove(tuner);
         parametersBeingTuned.remove(parameterName);
      };
      tuner.setCloseHandler(closeHandler);
      tuningBox.getChildren().add(tuner);
   }

   public void setRegistries(List<GuiRegistry> registries)
   {
      tuningBox.getChildren().clear();
      parametersBeingTuned.clear();
      tunerMap.clear();

      List<GuiParameter> allParameters = new ArrayList<>();
      registries.stream().forEach(registry -> allParameters.addAll(registry.getAllParameters()));
      allParameters.stream().forEach(parameter -> {
         Tuner tuner = new Tuner(parameter);
         tunerMap.put(parameter.getUniqueName(), tuner);
      });
   }

   public Map<String, Tuner> getTunerMap()
   {
      return tunerMap;
   }
}
