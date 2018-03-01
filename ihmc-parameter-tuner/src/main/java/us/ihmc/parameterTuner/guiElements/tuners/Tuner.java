package us.ihmc.parameterTuner.guiElements.tuners;


import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;
import javafx.scene.text.Text;
import us.ihmc.commons.PrintTools;
import us.ihmc.javaFXToolkit.TextFormatterTools;
import us.ihmc.parameterTuner.ParameterTuningTools;
import us.ihmc.parameterTuner.guiElements.GuiParameter;

public class Tuner extends VBox
{
   private static final int MAX_DESCRIPTION_CHARACTERS = 255;

   private Label name;
   private TextField description;
   private Button remove;
   private InputNode inputNode;

   public Tuner(GuiParameter parameter)
   {
      setupNode();

      setId("tuner-window");
      name.setId("parameter-name-in-tuner");

      name.setText(parameter.getName());
      description.setText(parameter.getCurrentDescription());
      description.setTextFormatter(TextFormatterTools.maxLengthTextFormatter(MAX_DESCRIPTION_CHARACTERS));
      ParameterTuningTools.addThreadSafeListeners(description, () -> parameter.setDescription(description.getText()));

      parameter.addChangedListener(p -> {
         // This listener will be triggered by an external change and is called from the animation timer.
         description.setText(parameter.getCurrentDescription());
      });

      switch (parameter.getType())
      {
      case "DoubleParameter":
         DoubleTuner doubleTuner = new DoubleTuner(parameter);
         getChildren().add(doubleTuner);
         inputNode = doubleTuner;
         break;
      case "IntegerParameter":
         IntegerTuner integerTuner = new IntegerTuner(parameter);
         getChildren().add(integerTuner);
         inputNode = integerTuner;
         break;
      case "BooleanParameter":
         BooleanTuner booleanTuner = new BooleanTuner(parameter);
         getChildren().add(booleanTuner);
         inputNode = booleanTuner;
         break;
      case "EnumParameter":
         EnumTuner enumTuner = new EnumTuner(parameter);
         getChildren().add(enumTuner);
         inputNode = enumTuner;
         break;
      default:
         PrintTools.info("Implement me.");
      }
   }

   private void setupNode()
   {
      setSpacing(10.0);
      setMaxHeight(Double.NEGATIVE_INFINITY);
      setMaxWidth(Double.NEGATIVE_INFINITY);
      setPrefWidth(700);

      name = new Label();
      remove = new Button("Remove");
      description = new TextField();
      HBox.setHgrow(name, Priority.ALWAYS);
      HBox.setHgrow(description, Priority.ALWAYS);

      HBox parameterInfoBox = new HBox();
      parameterInfoBox.setSpacing(10.0);
      parameterInfoBox.setAlignment(Pos.CENTER_LEFT);
      parameterInfoBox.setPadding(new Insets(5.0, 5.0, 0.0, 5.0));
      parameterInfoBox.getChildren().add(remove);
      parameterInfoBox.getChildren().add(name);
      getChildren().add(parameterInfoBox);

      HBox parameterDescriptionBox = new HBox();
      parameterDescriptionBox.setSpacing(10.0);
      parameterDescriptionBox.setAlignment(Pos.CENTER_LEFT);
      parameterDescriptionBox.setPadding(new Insets(5.0, 5.0, 0.0, 5.0));
      parameterDescriptionBox.getChildren().add(new Text("Description"));
      parameterDescriptionBox.getChildren().add(description);
      getChildren().add(parameterDescriptionBox);
   }

   public void setCloseHandler(EventHandler<ActionEvent> closeHandler)
   {
      remove.setOnAction(closeHandler);
   }

   public Node getSimpleInputNode()
   {
      return inputNode.getSimpleInputNode();
   }

}
