package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Utility class to interface to a LimelightFX */
public class LimelightFX extends SubsystemBase {
	/**
	 * An RGBA color
	 * 
	 * Colors are 0-255
	 */
	public static final class Color {
		public static final Color BLACK = new Color(255);
		public static final Color WHITE = new Color(255);
		public static final Color RED = new Color(255, 0, 0);
		public static final Color GREEN = new Color(0, 255, 0);
		public static final Color BLUE = new Color(0, 0, 255);

		public final int r;
		public final int g;
		public final int b;
		public final int a;

		public Color(int r, int g, int b, int a) {
			this.r = MathUtil.clamp(r, 0, 255);
			this.g = MathUtil.clamp(g, 0, 255);
			this.b = MathUtil.clamp(b, 0, 255);
			this.a = MathUtil.clamp(a, 0, 255);
		}

		public Color(int r, int g, int b) { this(r, g, b, 0); }

		public Color(int light) { this(light, light, light, 255); }

		public Color() { this(255); }

		/** Create a new color with this one's color and a new alpha */
		public Color withAlpha(int a) {
			return new Color(this.r, this.g, this.b, a);
		}

		@Override
		public String toString() {
			return String.format(
				"0x%x%x%x%x",
				this.a,
				this.r,
				this.g,
				this.b
			);
		}
	}

	private static interface Behavior {
		public int getID();

		public String getParams();
	}

	// BEHAVIORS //

	public static class ChevronsBehavior implements Behavior {
		public enum Direction {
			Up(0),
			Down(2),
			Left(1),
			Right(3);
			
			public final int value;
			
			private Direction(int value) { this.value = value; }
		}

		/** The Color for the A chevrons */
		public Color colorA;
		/** The Color for the B chevrons */
		public Color colorB;
		/** The thickness for the A chevrons in pixels */
		public int thicknessA;
		/** The thickness for the B chevrons in pixels */
		public int thicknessB;
		/**
		 * The speed the chevrons should move at
		 * 
		 * Measured in pixels per tick (100 ticks/sec)
		 */
		public int speed;
		/** The direction the chevrons should move in */
		public Direction dir;
		
		@Override
		public int getID() { return 5; }

		@Override
		public String getParams() {
			return String.format(
				"%s %s %d %d %d %d",
				this.colorA,
				this.colorB,
				this.thicknessA,
				this.thicknessB,
				this.speed * 255,
				this.dir.value
			);
		}
	}

	// IMPLEMENTATION //

	/** The SerialPort connected to the LimelightFX **/
	private final SerialPort fx;

	/** The current behavior **/
	private Behavior active;

	/**
	 * Creates an interface to a LimelightFX
	 * 
	 * @param port The serial port to communicate over
	 */
	public LimelightFX(SerialPort.Port port) {
		this.fx = new SerialPort(115200, port);
	}

	/** Puts the LimelightFX into addressable mode, where no behaviors are active and the only painting is done by the user **/
	public void addressable() {
		this.fx.writeString("behavior 0 0");
		this.active = null;
	}

	/**
	 * Starts a behavior and sets its parameters
	 * 
	 * @param behavior The behavior to start
	 */
	public void setBehavior(Behavior behavior) {
		this.fx.writeString(String.format("behavior 0 %d %s", behavior.getID(), behavior.getParams()));
		this.active = behavior;
	}

	/** Updates the LimelightFX with the current parameters of the active behavior, if there is one **/
	public void update() {
		if(this.active == null) return;
		this.fx.writeString(String.format("param 0 %s",  this.active.getParams()));
	}

	/** Returns the current active behavior, if there is one **/
	public Behavior active() { return this.active; }

	/**
	 * Paint a single pixel
	 * 
	 * @apiNote Can be used at any time but might be painted over by a behavior if you aren't in addressable mode
	 * 
	 * @param x X position (0 is left)
	 * @param y Y position (0 is top)
	 * @param col Color to paint
	 */
	public void px(int x, int y, Color col) {
		this.fx.writeString(String.format("px %d %d %s", x, y, col));
	}

	/**
	 * Paint the entire FX with the color
	 * 
	 * @apiNote Can be used at any time but might be painted over by a behavior if you aren't in addressable mode
	 * 
	 * @param col Color to paint
	 */
	public void fill(Color col) {
		this.fx.writeString(String.format("fill %s", col));
	}

	/**
	 * Clear the FX
	 * 
	 * 
	 */
	public void clear() { this.fill(Color.BLACK); }
}
