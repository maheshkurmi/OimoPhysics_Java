package demo.common;

/**
 * ...
 */
public class UserInput {
	public double mouseX;
	public double mouseY;
	public double pmouseX;
	public double pmouseY;
	public boolean mouseL;
	public boolean mouseR;
	public boolean pmouseL;
	public boolean pmouseR;

	public boolean[] keyboard;
	public  boolean[] pkeyboard;
	public static int KEYBOARD_LENGTH	=256;//(default, never):Int = 256;
	public static int KEYCODE_LEFT		=37;//(default, never):Int = 37;
	public static int KEYCODE_UP		=38;//(default, never):Int = 38;
	public static int KEYCODE_RIGHT		=39;//(default, never):Int = 39;
	public static int KEYCODE_DOWN		=40;//(default, never):Int = 40;
	public static int KEYCODE_ENTER		=13;//(default, never):Int = 13;
	public static int KEYCODE_RETURN	=13;//(default, never):Int = 13;

	public UserInput() {
		pmouseX = 0;
		pmouseY = 0;
		pmouseL = false;
		pmouseR = false;
		mouseX = 0;
		mouseY = 0;
		mouseL = false;
		mouseR = false;
		pkeyboard = new boolean[KEYBOARD_LENGTH];
		keyboard = new boolean[KEYBOARD_LENGTH];
		for (int i=0;i<KEYBOARD_LENGTH;i++) {
			pkeyboard[i] = false;
			keyboard[i] = false;
		}
	}

	public boolean isKeyPressed(int code) {
		if (code < 0 || code >= KEYBOARD_LENGTH) return false;
		return !pkeyboard[code] && keyboard[code];
	}
	
	public boolean isKeyPressed(String ch) {
		if (ch != null) {
			return isKeyPressed(ch.charAt(0));
		}
		return false;
	}
}