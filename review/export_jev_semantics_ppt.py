#!/usr/bin/env python3
"""Package generated SVG slides into a 16:9 Impress/PPTX presentation via UNO."""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import uno
from com.sun.star.beans import PropertyValue
from com.sun.star.awt import Point, Size


ROOT = Path(__file__).resolve().parent
SVG_DIR = ROOT / "jev-semantics-slides"
ODP = ROOT / "jev-state-semantics-constraint-architecture.odp"
PPTX = ROOT / "jev-state-semantics-constraint-architecture.pptx"


def prop(name, value):
    p = PropertyValue()
    p.Name = name
    p.Value = value
    return p


def connect(port):
    local = uno.getComponentContext()
    resolver = local.ServiceManager.createInstanceWithContext(
        "com.sun.star.bridge.UnoUrlResolver", local
    )
    for _ in range(80):
        try:
            return resolver.resolve(
                f"uno:socket,host=127.0.0.1,port={port};urp;StarOffice.ComponentContext"
            )
        except Exception:
            time.sleep(0.15)
    raise RuntimeError("Unable to connect to headless LibreOffice")


def main():
    slides = sorted(SVG_DIR.glob("*.svg"))
    if not slides:
        raise SystemExit("No SVG slides found; run generate_jev_semantics_slides.py first")
    port = 20837
    profile = Path(tempfile.mkdtemp(prefix="jev-ppt-profile-"))
    office = subprocess.Popen(
        [
            "soffice",
            "--headless",
            f"--accept=socket,host=127.0.0.1,port={port};urp;StarOffice.ServiceManager",
            f"-env:UserInstallation={profile.as_uri()}",
            "--nologo",
            "--nodefault",
            "--nofirststartwizard",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        ctx = connect(port)
        smgr = ctx.ServiceManager
        desktop = smgr.createInstanceWithContext("com.sun.star.frame.Desktop", ctx)
        doc = desktop.loadComponentFromURL("private:factory/simpress", "_blank", 0, ())
        pages = doc.getDrawPages()
        while pages.getCount() > 1:
            pages.remove(pages.getByIndex(pages.getCount() - 1))
        provider = smgr.createInstanceWithContext("com.sun.star.graphic.GraphicProvider", ctx)
        for i, svg in enumerate(slides):
            page = pages.getByIndex(0) if i == 0 else pages.insertNewByIndex(i)
            page.Width = 33867
            page.Height = 19050
            page.BorderLeft = page.BorderRight = page.BorderTop = page.BorderBottom = 0
            graphic = provider.queryGraphic((prop("URL", svg.resolve().as_uri()),))
            shape = doc.createInstance("com.sun.star.drawing.GraphicObjectShape")
            shape.Graphic = graphic
            shape.Position = Point(0, 0)
            shape.Size = Size(33867, 19050)
            page.add(shape)
        doc.getDocumentProperties().Title = "JevPilot 状态语义、自动驾驶闭环与约束架构"
        doc.getDocumentProperties().Subject = "位置、感知、候选、Jev 选择、物理 step、约束分布与治理"
        doc.storeAsURL(ODP.resolve().as_uri(), (prop("FilterName", "impress8"), prop("Overwrite", True)))
        doc.storeAsURL(
            PPTX.resolve().as_uri(),
            (prop("FilterName", "Impress MS PowerPoint 2007 XML"), prop("Overwrite", True)),
        )
        doc.close(True)
        print(f"Wrote {PPTX}")
        print(f"Wrote {ODP}")
    finally:
        office.terminate()
        try:
            office.wait(timeout=5)
        except subprocess.TimeoutExpired:
            office.kill()


if __name__ == "__main__":
    main()
