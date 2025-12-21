#!/usr/bin/env python3
"""
Interactive tool to mark trunk locations by clicking on images
- Click to place trunk center
- Mouse wheel or +/- keys to adjust radius
- Right-click to remove last trunk
- Press 's' to save
- Press 'c' to clear all
- Press 'q' or ESC to quit
"""

import cv2
import numpy as np
import json
import os

class TrunkMarker:
    def __init__(self, image_path):
        self.image_path = image_path
        self.image = cv2.imread(image_path)
        if self.image is None:
            raise ValueError(f"Could not load image: {image_path}")
        
        self.display_image = self.image.copy()
        self.trunks = []  # List of (x, y, radius)
        self.current_radius = 6  # Default radius
        self.drawing = False
        self.current_pos = None
        self.show_coords = True  # Show coordinate overlay
        self.show_info_panel = True  # Show info panel
        
        # Window setup
        self.window_name = f"Mark Trunks: {os.path.basename(image_path)}"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        
        # Instructions
        self.show_instructions()
    
    def show_instructions(self):
        """Display instructions on the image"""
        instructions = [
            "LEFT CLICK: Mark trunk center",
            "MOUSE WHEEL or +/-: Adjust radius",
            "RIGHT CLICK: Remove last trunk",
            "Press 's': Save and continue",
            "Press 'c': Clear all",
            "Press 'i': Toggle info panel",
            "Press 'h': Toggle crosshair",
            "Press 'q' or ESC: Quit"
        ]
        print("\n" + "="*60)
        print(f"Marking trunks in: {os.path.basename(self.image_path)}")
        print("="*60)
        for inst in instructions:
            print(f"  {inst}")
        print("="*60 + "\n")
    
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events"""
        self.current_pos = (x, y)
        
        if event == cv2.EVENT_LBUTTONDOWN:
            # Add trunk at clicked position
            self.trunks.append((x, y, self.current_radius))
            print(f"  Marked trunk #{len(self.trunks)}: center=({x:3d}, {y:3d}), radius={self.current_radius}")
            self.update_display()
        
        elif event == cv2.EVENT_RBUTTONDOWN:
            # Remove last trunk
            if self.trunks:
                removed = self.trunks.pop()
                print(f"  Removed trunk: center=({removed[0]:3d}, {removed[1]:3d}), radius={removed[2]}")
                self.update_display()
        
        elif event == cv2.EVENT_MOUSEWHEEL:
            # Adjust radius with mouse wheel
            if flags > 0:  # Scroll up
                self.current_radius = min(self.current_radius + 1, 50)
            else:  # Scroll down
                self.current_radius = max(self.current_radius - 1, 3)
            self.update_display()
        
        elif event == cv2.EVENT_MOUSEMOVE:
            # Update display to show preview
            self.update_display()
    
    def update_display(self):
        """Update the display image with all marked trunks"""
        self.display_image = self.image.copy()
        
        # Draw coordinate grid (optional, can be toggled)
        h, w = self.image.shape[:2]
        
        # Draw all marked trunks
        for idx, (tx, ty, tr) in enumerate(self.trunks):
            # Draw circle
            cv2.circle(self.display_image, (tx, ty), tr, (0, 255, 0), 2)
            cv2.circle(self.display_image, (tx, ty), 3, (0, 255, 0), -1)
            
            # Draw number and info
            label = f"#{idx+1}"
            info = f"r={tr}"
            cv2.putText(self.display_image, label, (tx + tr + 5, ty - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(self.display_image, info, (tx + tr + 5, ty + 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Draw preview circle at mouse position
        if self.current_pos:
            x, y = self.current_pos
            cv2.circle(self.display_image, (x, y), self.current_radius, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.circle(self.display_image, (x, y), 2, (0, 255, 255), -1)
            
            # Show coordinates and radius
            coord_text = f"({x}, {y})"
            radius_text = f"Radius: {self.current_radius}"
            cv2.putText(self.display_image, coord_text, (x + self.current_radius + 5, y - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(self.display_image, radius_text, (x + self.current_radius + 5, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Draw coordinate crosshair at mouse position
        if self.current_pos and self.show_coords:
            x, y = self.current_pos
            h, w = self.display_image.shape[:2]
            # Draw crosshair
            cv2.line(self.display_image, (x, 0), (x, h), (255, 255, 255), 1, cv2.LINE_AA)
            cv2.line(self.display_image, (0, y), (w, y), (255, 255, 255), 1, cv2.LINE_AA)
            # Draw coordinate labels at edges
            cv2.putText(self.display_image, str(x), (x + 5, 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            cv2.putText(self.display_image, str(y), (5, y - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Draw info panel at bottom-right corner (less intrusive)
        if self.show_info_panel:
            h, w = self.display_image.shape[:2]
            panel_w = 280
            panel_h = 100
            panel_x = w - panel_w - 10
            panel_y = h - panel_h - 10
            
            # Semi-transparent background
            overlay = self.display_image.copy()
            cv2.rectangle(overlay, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.7, self.display_image, 0.3, 0, self.display_image)
            
            info_y = panel_y + 20
            cv2.putText(self.display_image, f"Trunks: {len(self.trunks)} | Radius: {self.current_radius}", 
                       (panel_x + 10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            if self.current_pos:
                x, y = self.current_pos
                cv2.putText(self.display_image, f"Mouse: ({x}, {y})", 
                           (panel_x + 10, info_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(self.display_image, "Press 'i' to toggle info", 
                       (panel_x + 10, info_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            cv2.putText(self.display_image, "Press 's' to save, 'q' to quit", 
                       (panel_x + 10, info_y + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        cv2.imshow(self.window_name, self.display_image)
    
    def run(self):
        """Main loop"""
        self.update_display()
        
        while True:
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q') or key == 27:  # 'q' or ESC
                break
            elif key == ord('s'):  # Save
                return True
            elif key == ord('c'):  # Clear all
                self.trunks = []
                print("  Cleared all trunks")
                self.update_display()
            elif key == ord('+') or key == ord('='):  # Increase radius
                self.current_radius = min(self.current_radius + 1, 50)
                self.update_display()
            elif key == ord('-') or key == ord('_'):  # Decrease radius
                self.current_radius = max(self.current_radius - 1, 3)
                self.update_display()
            elif key == ord('h'):  # Toggle coordinate crosshair
                self.show_coords = not self.show_coords
                self.update_display()
            elif key == ord('i'):  # Toggle info panel
                self.show_info_panel = not self.show_info_panel
                self.update_display()
        
        return False
    
    def get_trunks(self):
        """Get list of marked trunks"""
        return self.trunks.copy()

def main():
    """Main function"""
    image_dir = "example_rgb_images"
    image_files = sorted([f for f in os.listdir(image_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png'))])
    
    if not image_files:
        print(f"No images found in {image_dir}")
        return
    
    os.makedirs("trunk_detection_results", exist_ok=True)
    
    all_trunk_data = {}
    
    print("="*80)
    print("Interactive Trunk Marker")
    print("="*80)
    print(f"\nFound {len(image_files)} images to process")
    print("\nYou will mark trunks for each image one by one.")
    print("After marking all trunks in an image, press 's' to save and move to next image.")
    print("Press 'q' to quit at any time.\n")
    
    print("Starting in 2 seconds... (close this window if you want to run it manually)")
    import time
    time.sleep(2)
    
    for img_file in image_files:
        image_path = os.path.join(image_dir, img_file)
        
        try:
            marker = TrunkMarker(image_path)
            should_continue = marker.run()
            
            if not should_continue:
                print("\nQuitting...")
                break
            
            trunks = marker.get_trunks()
            all_trunk_data[img_file] = trunks
            
            print(f"\n✓ Saved {len(trunks)} trunks for {img_file}")
            
            # Auto-continue to next image (user can quit with 'q' if needed)
            # No need to ask - just continue
            
            cv2.destroyAllWindows()
        
        except Exception as e:
            print(f"Error processing {img_file}: {e}")
            continue
    
    # Save all data
    if all_trunk_data:
        # Convert to JSON-serializable format
        json_data = {}
        for img_file, trunks in all_trunk_data.items():
            json_data[img_file] = [
                {"center": [int(x), int(y)], "radius": int(r)}
                for x, y, r in trunks
            ]
        
        json_path = "trunk_detection_results/manually_marked_trunks.json"
        with open(json_path, 'w') as f:
            json.dump(json_data, f, indent=2)
        
        print("\n" + "="*80)
        print("Marking Complete!")
        print("="*80)
        print(f"\nSaved {sum(len(t) for t in all_trunk_data.values())} total trunks")
        print(f"Results saved to: {json_path}")
        print("\nSummary:")
        for img_file, trunks in all_trunk_data.items():
            print(f"  {img_file}: {len(trunks)} trunks")
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

