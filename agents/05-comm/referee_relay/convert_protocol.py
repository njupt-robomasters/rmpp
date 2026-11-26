"""按原页顺序提取协议正文、表格和代码示例。"""

import re
import sys
from pathlib import Path

import pdfplumber


def clean(text):
    return text.replace("\uf06c", "•").replace("\uf0d8", "▸").strip()


def main(source):
    sections = [f"# {source.stem}"]
    with pdfplumber.open(source) as document:
        for page in document.pages:
            sections.append(f"## 第 {page.page_number} 页")
            if page.page_number == 1:
                sections.append(f"[封面：{source.stem}](<{source.name}#page=1>)")
                continue
            tables = [table for table in page.find_tables() if len(table.columns) >= 2]
            blocks = []
            for table in tables:
                rows = table.extract()
                lines = []
                for index, row in enumerate(rows):
                    cells = [clean(cell or "").replace("|", r"\|").replace("\n", "<br>") for cell in row]
                    lines.append("| " + " | ".join(cells) + " |")
                    if index == 0:
                        lines.append("| " + " | ".join(["---"] * len(row)) + " |")
                blocks.append((table.bbox[1], "\n" + "\n".join(lines) + "\n"))
            remaining = page.filter(lambda obj: obj["object_type"] != "char" or not any(
                left <= (obj["x0"] + obj["x1"]) / 2 < right and top <= (obj["top"] + obj["bottom"]) / 2 < bottom
                for left, top, right, bottom in (table.bbox for table in tables)))
            for line in remaining.extract_text_lines():
                text = clean(line['text'])
                if text and not re.fullmatch(r"(?:\d+\s*)?©\s*2026\s*大疆\s*版权所有(?:\s*\d+)?", text):
                    blocks.append((line['top'], text))
            sections.append("\n".join(text for _, text in sorted(blocks, key=lambda item: item[0])))
            page.close()
    source.with_suffix('.md').write_text("\n\n".join(sections).rstrip() + "\n", encoding='utf-8')


if __name__ == '__main__':
    main(Path(sys.argv[1]))
