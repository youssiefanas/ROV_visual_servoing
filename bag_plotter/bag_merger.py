import os
from PIL import Image

def merge_pngs_in_folder_horizontal(png_folder, output_path):
    """Merge all PNGs in a folder horizontally and save to output_path."""
    png_files = [f for f in os.listdir(png_folder) if f.endswith('.png')]
    if not png_files:
        print(f"No PNG files found in {png_folder}")
        return

    png_files.sort()  # Ensures a consistent order
    images = [Image.open(os.path.join(png_folder, f)) for f in png_files]

    # Compute total width and max height
    widths, heights = zip(*(img.size for img in images))
    total_width = sum(widths)
    max_height = max(heights)

    merged_image = Image.new('RGB', (total_width, max_height), (255,255,255))

    x_offset = 0
    for img in images:
        merged_image.paste(img, (x_offset, 0))
        x_offset += img.width

    merged_image.save(output_path)
    print(f"Saved merged image: {output_path}")

def merge_all_bag_pngs_horizontal():
    base_dir = os.path.join(os.getcwd(), "plot_result")
    for bagname in os.listdir(base_dir):
        bag_dir = os.path.join(base_dir, bagname)
        png_dir = os.path.join(bag_dir, "png")
        output_path = os.path.join(bag_dir, "merged.png")
        if os.path.isdir(png_dir):
            merge_pngs_in_folder_horizontal(png_dir, output_path)

if __name__ == '__main__':
    merge_all_bag_pngs_horizontal()
