package demo.common;

import java.util.function.Function;

/**
 * Defines a demo control with given text, keyboard mapping and action.
 */
public  class Control {
	public String keyText;
	public Function description;//(default, null):Void -> String;
	public int keyCode;//(default, null):Int;
	public Function onAction;//(default, null):Void -> Void;

	public  Control(String keyText, Function  description, int keyCode, Function onAction) {
		this.keyText = keyText;
		this.description = description;
		this.keyCode = keyCode;
		this.onAction = onAction;
	}

}