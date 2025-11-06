"""Capture MuJoCo scene renderings to image files.

This module provides utilities to load MJCF files and render them to images
with different visualization settings.
"""

import argparse
import logging
from pathlib import Path
import numpy as np

try:
    import mujoco
except ImportError:
    raise ImportError("MuJoCo is required for capture functionality. Install with: pip install mujoco")

logger = logging.getLogger(__name__)


def capture_scene(
    mjcf_path: Path,
    output_path: Path,
    width: int = 2000,
    height: int = 2000,
    enable_group2: bool = True,
    enable_group3: bool = False,
) -> None:
    """
    Capture a rendered image of the MuJoCo scene.
    
    Args:
        mjcf_path: Path to the MJCF XML file
        output_path: Path to save the output image
        width: Image width in pixels
        height: Image height in pixels
        enable_group2: Whether to enable geom group 2 (visual geometries)
        enable_group3: Whether to enable geom group 3 (collision geometries)
    """
    try:
        # Load the model
        model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        data = mujoco.MjData(model)
        
        # Ensure width and height don't exceed framebuffer limits
        # MuJoCo default framebuffer is 640x480, check model settings
        max_width = getattr(model.vis.global_, 'offwidth', 640)
        max_height = getattr(model.vis.global_, 'offheight', 480)
        
        # Adjust if requested size exceeds limits
        if width > max_width or height > max_height:
            logger.warning(f"Requested size {width}x{height} exceeds framebuffer {max_width}x{max_height}, adjusting...")
            scale = min(max_width / width, max_height / height)
            width = int(width * scale)
            height = int(height * scale)
            logger.info(f"Adjusted size to {width}x{height}")
        
        # Create renderer
        renderer = mujoco.Renderer(model, height=height, width=width)
        
        # Set camera to a good viewing angle
        # Use free camera with automatic framing
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, camera)
        
        # Set camera distance based on model size
        # Calculate bounding box
        mujoco.mj_forward(model, data)
        # extent = np.max(model.stat.extent)
        # camera.distance = extent * 1.0  # Adjust multiplier for better framing
        camera.azimuth = 150  # Rotate around z-axis
        camera.elevation = -15  # Look down at the model

        # Configure visualization options
        scene_option = mujoco.MjvOption()
        mujoco.mjv_defaultOption(scene_option)
        
        # Configure geom groups
        # Group 0: typically static world geometry
        # Group 1: typically dynamic bodies
        # Group 2: typically visual geometries
        # Group 3: typically collision geometries
        scene_option.geomgroup[0] = 1
        scene_option.geomgroup[1] = 1
        scene_option.geomgroup[2] = 1 if enable_group2 else 0
        scene_option.geomgroup[3] = 1 if enable_group3 else 0
        
        # Update the scene
        renderer.update_scene(data, camera=camera, scene_option=scene_option)
        
        # Render and save
        pixels = renderer.render()
        
        # Save using PIL for better format support
        try:
            from PIL import Image
            image = Image.fromarray(pixels)
            image.save(output_path)
            logger.info(f"Saved image to: {output_path}")
        except ImportError:
            # Fallback to matplotlib if PIL is not available
            import matplotlib.pyplot as plt
            plt.imsave(output_path, pixels)
            logger.info(f"Saved image to: {output_path} (using matplotlib)")
        
        # Clean up
        renderer.close()
        
    except Exception as e:
        logger.error(f"Failed to capture scene: {e}")
        raise


def capture_robot_images(mjcf_path: Path) -> None:
    """
    Capture standard robot visualization images.
    
    Creates two images:
    1. robot_name.png - Standard visualization with visual geometries
    2. collision.png - Collision geometry visualization
    
    Args:
        mjcf_path: Path to the MJCF XML file
    """
    mjcf_path = Path(mjcf_path)
    
    if not mjcf_path.exists():
        raise FileNotFoundError(f"MJCF file not found: {mjcf_path}")
    
    output_dir = mjcf_path.parent
    robot_name = mjcf_path.stem
    
    print(f"Capturing images for {robot_name}...")
    
    # Capture standard view (visual geometries)
    standard_output = output_dir / f"{robot_name}.png"
    print(f"  Rendering standard view...")
    capture_scene(
        mjcf_path=mjcf_path,
        output_path=standard_output,
        enable_group2=True,  # Enable visual geometries
        enable_group3=False,  # Disable collision geometries
    )
    print(f"  ✓ Saved: {standard_output}")
    
    # Capture collision view
    collision_output = output_dir / "collision.png"
    print(f"  Rendering collision view...")
    capture_scene(
        mjcf_path=mjcf_path,
        output_path=collision_output,
        enable_group2=False,  # Disable visual geometries
        enable_group3=True,   # Enable collision geometries
    )
    print(f"  ✓ Saved: {collision_output}")
    
    print("✅ Image capture complete!")


def main():
    """Command-line interface for capture functionality."""
    parser = argparse.ArgumentParser(
        description="Capture MuJoCo scene renderings to image files.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Capture standard images for a robot
  %(prog)s robot.xml
  
  # Custom output with specific dimensions
  %(prog)s robot.xml --output custom.png --width 3840 --height 2160
  
  # Show only collision geometries
  %(prog)s robot.xml --output collision.png --no-visual --collision
        """
    )
    
    parser.add_argument(
        "mjcf_path",
        type=str,
        help="Path to the MJCF XML file"
    )
    parser.add_argument(
        "--output",
        type=str,
        help="Output image path (default: robot_name.png and collision.png in same directory)"
    )
    parser.add_argument(
        "--width",
        type=int,
        default=2000,
        help="Image width in pixels (default: 2000)"
    )
    parser.add_argument(
        "--height",
        type=int,
        default=2000,
        help="Image height in pixels (default: 2000)"
    )
    parser.add_argument(
        "--no-visual",
        action="store_true",
        help="Disable visual geometries (geom group 2)"
    )
    parser.add_argument(
        "--collision",
        action="store_true",
        help="Enable collision geometries (geom group 3)"
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Set logging level (default: INFO)"
    )
    
    args = parser.parse_args()
    
    # Setup logging
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format='%(levelname)s: %(message)s'
    )
    
    mjcf_path = Path(args.mjcf_path)
    
    if args.output:
        # Custom single output
        output_path = Path(args.output)
        capture_scene(
            mjcf_path=mjcf_path,
            output_path=output_path,
            width=args.width,
            height=args.height,
            enable_group2=not args.no_visual,
            enable_group3=args.collision,
        )
    else:
        # Standard two-image output
        capture_robot_images(mjcf_path)


if __name__ == "__main__":
    main()
