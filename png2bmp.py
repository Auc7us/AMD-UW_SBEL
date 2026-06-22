#!/usr/bin/env python3

import argparse
from pathlib import Path
from PIL import Image


def main():
    parser = argparse.ArgumentParser(
        description="Convert an image to 8-bit grayscale BMP for Chrono heightmaps."
    )

    parser.add_argument("input", help="Input image path, for example heightmap.png")
    parser.add_argument("output", help="Output BMP path, for example heightmap.bmp")
    parser.add_argument(
        "--flip-y",
        action="store_true",
        help="Flip image vertically if your terrain appears inverted in Chrono.",
    )
    parser.add_argument(
        "--invert",
        action="store_true",
        help="Invert height values. Useful if craters appear as hills.",
    )

    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)

    img = Image.open(input_path)

    # Convert to true 8-bit grayscale.
    # BMP mode "L" means one channel, 8 bits per pixel.
    img = img.convert("L")

    if args.flip_y:
        img = img.transpose(Image.Transpose.FLIP_TOP_BOTTOM)

    if args.invert:
        img = Image.eval(img, lambda p: 255 - p)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    img.save(output_path, format="BMP")

    print(f"Input:  {input_path}")
    print(f"Output: {output_path}")
    print(f"Size:   {img.width} x {img.height}")
    print("Mode:   8-bit grayscale BMP")
    print("Done.")


if __name__ == "__main__":
    main()