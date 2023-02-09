package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.OIBase;
import frc.robot.subsystems.Log;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.Telemetry.TelemetryData;

public class POVSelector extends CommandBase {
    // TODO: improve tree creation, it's really ugly
    public static final class Tree {
        public static enum Discriminator {
            // Another layer of selection
            Select,

            // Leaf node, returns a value
            Leaf,
            
            // Cancel POVSelect
            Cancel,
        }

        public final String name;

        public final Discriminator discrim;

        public final Object leaf;

        public final Tree up;
        public final Tree right;
        public final Tree down;
        public final Tree left;

        public Tree() {
            this.name = "<exit>";

            this.discrim = Discriminator.Cancel;

            this.leaf = null;

            this.up = null;
            this.right = null;
            this.down = null;
            this.left = null;
        }

        public Tree(String name, Object leaf) {
            this.name = name;

            this.discrim = Discriminator.Leaf;

            this.leaf = leaf;

            this.up = null;
            this.right = null;
            this.down = null;
            this.left = null;
        }

        public Tree(String name, Tree up, Tree right, Tree down, Tree left) {
            this.name = name;

            this.discrim = Discriminator.Select;

            this.leaf = null;

            this.up = up;
            this.right = right;
            this.down = down;
            this.left = left;
        }
        
        public Tree pov(int pov) {
            if(pov == 0) return this.up;
            else if(pov == 90) return this.right;
            else if(pov == 180) return this.down;
            else if(pov == 270) return this.left;
            else return null;
        }

        public int totalDepth() {
            if(this.discrim != Discriminator.Select) return 1;

            return Math.max(
                Math.max(
                    this.up.totalDepth(),
                    this.right.totalDepth()
                ),
                Math.max(
                    this.down.totalDepth(),
                    this.left.totalDepth()
                )
            );
        }
    }

    private final TelemetryData telem = Telemetry.track("POV Selector", () -> {
        if(!this.isScheduled()) return "<inactive>";
        
        return String.join(" > ", this.state.stream().map(tree -> tree.name).toArray(String[]::new)) + " > ?";
    }, true);
    
    private final OIBase oi;
    private final Tree root;

    private ArrayList<Tree> state;
    private boolean pressed;

    public Consumer<String[]> hook;
    public BiConsumer<Object, String[]> finished;
	
	public POVSelector(OIBase oi, Consumer<String[]> hook, BiConsumer<Object, String[]> finished, Tree root) {
        this.oi = oi;
        this.hook = hook;
        this.finished = finished;
        this.root = root;
        this.state = new ArrayList<Tree>(this.root.totalDepth());

        this.telem.publish();
	}

	@Override
	public void initialize() {
        this.state.clear();
        this.state.add(this.root);
        
        Log.writeln("POV Selector: initialized");

        this.telem.publish();
	}

    @Override
    public void execute() {
        int pov = this.oi.controller.getPOV();

        if(pov == -1) {
            this.pressed = false;
            return;
        }
        
        if(this.pressed) return;

        this.pressed = true;

        Tree next = this.state.get(this.state.size() - 1).pov(pov);
        
        if(next != null) {
            this.state.add(next);
            
            String[] names = this.state.stream().map(tree -> tree.name).toArray(String[]::new);

            if(this.hook != null) this.hook.accept(names);

            if(next.discrim == Tree.Discriminator.Leaf) {
                if(this.finished != null) this.finished.accept(next.leaf, names);
                this.cancel();
            } else if(next.discrim == Tree.Discriminator.Cancel) {
                this.cancel();
            }

            this.telem.publish();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Log.writeln("POV Selector: end");
        
        this.telem.publish();
    }

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}
}
