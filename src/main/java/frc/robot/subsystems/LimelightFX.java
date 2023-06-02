package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

import java.awt.*;
import java.awt.image.*;
import java.util.Optional;

/** Utility class to interface to a LimelightFX */
public class LimelightFX extends SubsystemBase {
	/**
	 * An RGB color
	 *
	 * Colors are 0-255
	 */
	public static final class Color {
		public static final Color BLACK = new Color(0);
		public static final Color WHITE = new Color(255);
		public static final Color RED = new Color(255, 0, 0);
		public static final Color GREEN = new Color(0, 255, 0);
		public static final Color BLUE = new Color(0, 0, 255);

		public final int r;
		public final int g;
		public final int b;

		/** Creates a Color from RGB values */
		public Color(final int r, final int g, final int b) {
			this.r = MathUtil.clamp(r, 0, 255);
			this.g = MathUtil.clamp(g, 0, 255);
			this.b = MathUtil.clamp(b, 0, 255);
		}

		/** Creates a Color from white value */
		public Color(final int light) { this(light, light, light); }

		@Override
        public boolean equals(final Object other) {
			if(this != other) return false;
			if(this instanceof Color) {
				final Color col = (Color)other;
				return this.r == col.r
					&& this.g == col.g
					&& this.b == col.b;
			} else if(other instanceof int[]) {
				final int[] arr = (int[])other;
				return this.r == arr[0]
					&& this.g == arr[1]
					&& this.b == arr[2];
			}
			return false;
		}

		@Override
		public String toString() {
			return String.format(
				"0x%02x%02x%02x",
				this.r,
				this.g,
				this.b
			);
		}
	}

	/**
	 * A guard that will submit it's commands to the LimelightFX when closed
	 */
	public class BurstGuard implements AutoCloseable {
		StringBuilder queue = new StringBuilder();

		BurstGuard() {}

		void queue(final String data) { this.queue.append(data); }

		@Override
		public void close() {
			LimelightFX.this.write(this.queue.toString());
			LimelightFX.this.burst = null;
		}
	}

	/**
	 * A reference to some T that implements AutoCloseable. When this class is closed,
	 * it checks if it's value is owned, it will only close the value if the reference is
	 * counted as owned
	 */
	public static class GuardRef implements AutoCloseable {
		public final boolean owned;
		public final BurstGuard ref;

		private GuardRef(final boolean owned, final BurstGuard ref) {
			this.owned = owned;
			this.ref = ref;
		}

		/** Creates an owned reference that will close it's referent when closed */
		public static GuardRef owned(final BurstGuard ref) { return new GuardRef(true, ref); }
		/** Creates an owned reference that will NOT close it's referent when closed */
		public static GuardRef borrowed(final BurstGuard ref) { return new GuardRef(false, ref); }

		@Override
		public void close() {
			if(this.owned) this.ref.close();
		}
	}

	public static interface Behavior {
		public int getID();

		public String getParams();
	}

	public static class Behaviors {
		public static class ChevronsBehavior implements Behavior {
			public enum Direction {
				Up(0),
				Down(2),
				Left(1),
				Right(3);

				public final int value;

				private Direction(final int value) { this.value = value; }
			}

			/** The Color for the A chevrons */
			public Color colorA = new Color(255, 127, 255);
			/** The Color for the B chevrons */
			public Color colorB = new Color(127, 0, 255);
			/** The thickness for the A chevrons in pixels */
			public int thicknessA = 4;
			/** The thickness for the B chevrons in pixels */
			public int thicknessB = 2;
			/**
			 * The speed the chevrons should move at
			 *
			 * Measured in pixels per tick (100 ticks/sec)
			 */
			public int speed = 1;
			/** The direction the chevrons should move in */
			public Direction dir = Direction.Up;

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

		public static class CountdownBehavior implements Behavior {
			/** The Color for the countdown */
			//public Color colorA = new Color(255);
            public Color colorA = new Color(255);
			/** The total countdown time in milliseconds */
			public int ms = 15000;
			/** Whether the countdown should stop when it hits zero */
			public boolean stopAtZero = true;
			/** Whether the countdown should be paused */
			public boolean paused = false;

			@Override
			public int getID() { return 11; }

			@Override
			public String getParams() {
				return String.format(
					"%s %d %d %d",
					this.colorA,
					this.ms,
					this.stopAtZero ? 1 : 0,
					this.paused ? 1 : 0
				);
			}
		}
	}

	// IMPLEMENTATION //

	/** The SerialPort connected to the LimelightFX **/
	private SerialPort serial;

	/** The current behavior **/
	private Behavior active;

	/** The queued commands */
	private BurstGuard burst;

	/**
	 * Creates an interface to a LimelightFX
	 *
	 * @param port The serial port to communicate over
	 */
	public LimelightFX(final SerialPort.Port port) {
        if(Constants.LimelightFXConstants.disable) return;

		this.serial = new SerialPort(115200, port);
		this.serial.reset();
		this.serial.enableTermination();
		this.serial.setTimeout(0.5);
		this.serial.setWriteBufferMode(WriteBufferMode.kFlushOnAccess);
		this.write("init 0\nstart\n");
		this.addressable();
	}

    /**
     * Writes data to the port and disables the FX if no reply
     */
    // todo: test byte result
    private void write(final String data) {
        if(this.serial == null) return;

        final int n = this.serial.writeString(data);

        if(n == 0) this.disable("no response");
    }

	/** Writes to the current BurstGuard if it exists or writes directly to serial */
	private void writeMaybeQueue(final String data) {
		if(this.burst != null) this.burst.queue(data);
		else this.write(data);
	}

    /**
     * Disable the FX
     */
	public void disable(final String reason) {
        if(this.serial == null) return;

        this.serial.close();
        this.serial = null;
        Log.warning("LimelightFX disabled: " + reason);
    }

    /**
	 * Begins a command burst. All addressable draw commands issued while the burst
	 * is open are written into a queue and flushed when the CommandBurst is closed.
	 * (Use a try-with-resources block or manually call {@code guard.close()})
	 *
	 * @return A reference to a BurstGuard that commands are written into, close it to submit commands.
	 * If a guard does not exist, creates one and returns an owned reference to it
	 * If a guard already exists, a borrowed reference to it is returned
	 */
	public GuardRef burst() {
		if(this.burst != null) return GuardRef.borrowed(this.burst);

		final BurstGuard guard = new BurstGuard();
		this.burst = guard;
		return GuardRef.owned(guard);
	}

	/** Puts the LimelightFX into addressable mode, where no behaviors are active and the only painting is done by the user **/
	public LimelightFX addressable() {
		this.write("behavior 0 0\n");
		this.active = null;
		return this;
	}

	/**
	 * Starts a behavior and sets its parameters
	 *
	 * @param behavior The behavior to start
	 */
	public LimelightFX behavior(final Behavior behavior) {
		this.write(String.format("behavior 0 %d %s\n", behavior.getID(), behavior.getParams()));
		this.active = behavior;
		return this;
	}

	/** Updates the LimelightFX with the current parameters of the active behavior, if there is one **/
	public LimelightFX update() {
		if(this.active == null) return this;
		this.write(String.format("param 0 %s\n", this.active.getParams()));
		return this;
	}

	/** Returns the current active behavior, if there is one **/
	public Behavior active() { return this.active; }

	// ADDRESSABLE //

	/**
	 * Sends a special signal instructing the display to "vsync" and be done accepting
	 * commands this frame, use this if you have calls stacking up faster than the
	 * display can process them
	 */
	public LimelightFX vsync() {
		this.writeMaybeQueue("~\n");
		return this;
	}

	/**
	 * Paint a single pixel on the FX
	 *
	 * @param x X position (0 is left)
	 * @param y Y position (0 is top)
	 * @param col Color to paint
	 *
	 * @apiNote Can be used at any time but might be painted over by a behavior if you
	 * aren't in addressable mode
	 * @apiNote Will be written into a BurstGuard if one is present
	 */
	public LimelightFX px(final int x, final int y, final Color col) {
		this.writeMaybeQueue(String.format("px %d %d %s\n", x, y, col));
		return this;
	}

	/**
	 * Paint the entire FX with the color
	 *
	 * @param col Color to paint
	 *
	 * @apiNote Can be used at any time but might be painted over by a behavior if you
	 * aren't in addressable mode
	 */
	public LimelightFX fill(final Color col) {
		this.writeMaybeQueue(String.format("fill %s\n", col));
		return this;
	}

	/**
	 * Clear the FX
	 *
	 * @apiNote Can be used at any time but might be painted over by a behavior if you
	 * aren't in addressable mode
	 */
	public LimelightFX clear() {
		this.fill(Color.BLACK);
		return this;
	}

	/**
	 * Draw a box on the FX
	 *
	 * @param rect Rectangle to draw
	 * @param col Color to draw
	 * @param fill Whether the box should be filled
	 *
	 * @apiNote Can be used at any time but might be painted over by a behavior if you
	 * aren't in addressable mode
	 */
	public LimelightFX box(final Rectangle rect, final Color col, final boolean fill) {
		this.writeMaybeQueue(String.format(
			"box %d %d %d %d %s %d",
			rect.getMinX(),
			rect.getMinY(),
			rect.getMaxX(),
			rect.getMaxY(),
			col,
			fill ? '1' : '0'
		));
		return this;
	}

	private void octant(final int xc, final int yc, final int x, final int y, final Color col) {
		this.px(xc + x, yc + y, col);
		this.px(xc + x, yc - y, col);
		this.px(xc - x, yc + y, col);
		this.px(xc - x, yc - y, col);

		this.px(xc + y, yc + x, col);
		this.px(xc + y, yc - x, col);
		this.px(xc - y, yc + x, col);
		this.px(xc - y, yc - x, col);
	}

	/**
	 * Draw a circle on the FX
	 *
	 * @param xc Center x
	 * @param yc Center y
	 * @param r Radius
	 * @param col Color
	 */
	public LimelightFX circle(final int xc, final int yc, final int r, final Color col) {
		int x = 0;
		int y = r;

		int d = 3 - 2 * r;

		try(GuardRef guard = this.burst()) {
			this.octant(xc, yc, x, y, col);

			while(y >= x) {
				x++;

				if(d > 0) {
					y--;
					d = d + 4 * (x - y) + 10;
				} else d = d + 4 * x + 6;

				this.octant(xc, yc, x, y, col);
			}
		}
		return this;
	}

	/**
	 * Paint an image onto the FX starting at the upper left-hand corner and clipping at the bottom-right
	 *
	 * @param image Image to draw
	 */
	public LimelightFX image(final BufferedImage image) {
		return this.image(image, Optional.empty());
	}

	/**
	 * Paint an image onto the FX starting at the upper left-hand corner and clipping at the bottom-right
	 *
	 * @param image Image to draw
	 * @param alphaMask Mask color that should indicate pixels that should *not* be drawn
	 */
	public LimelightFX image(final BufferedImage image, final Optional<Color> alphaMask) {
        if(image == null) return this;
		final Rectangle dim = new Rectangle(image.getWidth(), image.getHeight());
		this.image(image, dim, dim, alphaMask);
		return this;
	}

	/**
	 * Paint an image onto the FX starting at the
	 * @param image
	 * @param from
	 * @param to
	 * @param alphaMask
	 */
	@SuppressWarnings("unlikely-arg-type")
	public LimelightFX image(final BufferedImage image, Rectangle from, final Rectangle to, final Optional<Color> alphaMask) {
        if(image == null) return this;

		from = from.intersection(new Rectangle(image.getWidth(), image.getHeight()));

		if(!from.intersects(to)) return this; // Short circuit if nothing actually needs to be drawn

        this.addressable();

		final boolean alpha = alphaMask.isPresent();
		final Color mask = alphaMask.orElse(null);

		final Raster raster = image.getData(from);

		final int[] pixel = new int[3];

		try(final GuardRef guard = this.burst()) {
			for(int y = 0; y < to.height; y++) {
				for(int x = 0; x < to.width; x++) {
					raster.getPixel(x, y, pixel);

					if(alpha && mask.equals(pixel)) continue; // Skip drawing pixels that fit the alpha mask

					// Avoid using the px method so we don't have to alloc a Color
					this.writeMaybeQueue(String.format(
						"px %d %d 0x%02x%02x%02x\n",
						x,
						y,
						pixel[0],
						pixel[1],
						pixel[2]
					));
				}
			}

            this.write("~\n");
		}

		return this;
	}

	/**
	 * Saves the state of the entire display to a save buffer.
	 *
	 * @param id Save buffer to save to (there are 4)
	 */
	public LimelightFX saveFrame(final int id) {
		this.writeMaybeQueue(String.format("save %d\n", id));
		return this;
	}

	/**
	 * Loads a new state of the display from a save buffer.
	 *
	 * @param id Save buffer to load from (there are 4)
	 */
	public LimelightFX loadFrame(final int id) {
		this.writeMaybeQueue(String.format("load %d\n", id));
		return this;
	}
}
