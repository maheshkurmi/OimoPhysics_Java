package demo.gl2renderer;
import java.util.HashMap;

import com.jogamp.opengl.GL2;

public class OShader {
	GL2 gl;
	private int shaderProgram;
	private int vertexShader;
	private int fragmentShader;
	HashMap<String, Integer> uniformLocationMap;
	
	public OShader(GL2 gl) {
		this.gl = gl;
        //one vertex shader and one fragment shader.
        shaderProgram = gl.glCreateProgram();
		// OpenGL  retuns a index id to be stored for future reference.
        vertexShader = gl.glCreateShader(GL2.GL_VERTEX_SHADER);
        fragmentShader = gl.glCreateShader(GL2.GL_FRAGMENT_SHADER);

	}

	public void compile(String vertexSource, String fragmentSource) {
		uniformLocationMap = new HashMap<String, Integer>();
		compileShader(vertexShader, vertexSource);
		compileShader(fragmentShader, fragmentSource);
		gl.glAttachShader(shaderProgram, vertexShader);
		gl.glAttachShader(shaderProgram, fragmentShader);
		gl.glLinkProgram(shaderProgram);
//		if (!gl.getProgramParameter(shaderProgram, GL2.GL_LINK_STATUS)) {
//			trace(gl.glGetProgramInfoLog(shaderProgram));
//		}
	}

	public int getAttribIndex(String name) {
		return gl.glGetAttribLocation(shaderProgram, name);
	}

	public int getUniformLocation(String name) {
		if (uniformLocationMap.containsKey(name)) return uniformLocationMap.get(name);
		int location = gl.glGetUniformLocation(shaderProgram, name);
		uniformLocationMap.put(name, location);
		return location;
	}

	public int[] getAttribIndices(OVertexAttribute[] attribs) {
		int indices[] = new int[attribs.length];
		for (int i=0;i<attribs.length;i++) {
			OVertexAttribute attrib=attribs[i];
			indices[i]=getAttribIndex(attrib.name);
		}
		return indices;
	}

	public void use() {
		gl.glUseProgram(shaderProgram);
	}

	private void compileShader(int shader, String source) {
		String[] lines = new String[] { source };
        int[] lengths = new int[] { lines[0].length() };
        gl.glShaderSource(shader, lines.length, lines, lengths, 0);
        gl.glCompileShader(shader);

        //Check compile status.
        int[] compiled = new int[1];
        gl.glGetShaderiv(shader, GL2.GL_COMPILE_STATUS, compiled,0);
        if(compiled[0]!=0){
        	System.out.println("Horray!  shader compiled");
        }else {
            int[] logLength = new int[1];
            gl.glGetShaderiv(shader, GL2.GL_INFO_LOG_LENGTH, logLength, 0);

            byte[] log = new byte[logLength[0]];
            gl.glGetShaderInfoLog(shader, logLength[0], (int[])null, 0, log, 0);

            System.err.println("Error compiling the vertex shader: " + new String(log));
            System.exit(1);
        }

//        
//		gl.shaderSource(shader, source);
//		gl.compileShader(shader);
//		if (!gl.getShaderParameter(shader, GL.COMPILE_STATUS)) {
//			Browser.alert(gl.getShaderInfoLog(shader));
//		}
	}

}
