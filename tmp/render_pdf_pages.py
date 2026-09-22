from pathlib import Path

import pypdfium2 as pdfium


PDF_PATH = Path(r"G:\stm32_Project\test_projects\PowerPico\tmp\PowerPico_进度清单_STEP5更新_待核验.pdf")
OUTPUT_DIR = Path(r"G:\stm32_Project\test_projects\PowerPico\tmp\render_step5")


def main() -> None:
    """把临时PDF逐页转成PNG，供人工检查文档排版。"""
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    pdf = pdfium.PdfDocument(PDF_PATH)
    for index in range(len(pdf)):
        page = pdf[index]
        bitmap = page.render(scale=1.7)
        bitmap.to_pil().save(OUTPUT_DIR / f"page-{index + 1:02d}.png")
    print(f"pages={len(pdf)}")


if __name__ == "__main__":
    main()
