#!/usr/bin/env python

from chainercv.experimental.links import FCISResNet101
from chainercv.experimental.links import PSPNetResNet101
from chainercv.experimental.links import PSPNetResNet50
from chainercv.experimental.links import YOLOv2Tiny
from chainercv.links import FasterRCNNFPNResNet101
from chainercv.links import FasterRCNNFPNResNet50
from chainercv.links import FasterRCNNVGG16
from chainercv.links import SSD300
from chainercv.links import SSD512
from chainercv.links import YOLOv2
from chainercv.links import YOLOv3
from chainercv import utils

models = {
    FasterRCNNFPNResNet101: ['coco'],
    FasterRCNNFPNResNet50: ['coco'],
    FasterRCNNVGG16: ['voc0712'],
    FCISResNet101: ['coco'],
    PSPNetResNet101: ['ade20k'],
    PSPNetResNet50: ['ade20k'],
    SSD300: ['voc0712'],
    SSD512: ['voc0712'],
    YOLOv2: ['voc0712'],
    YOLOv2Tiny: ['voc0712'],
    YOLOv3: ['voc0712'],
}


def main():
    for cls, pretrained_models in models.items():
        for pretrained_model in pretrained_models:
            url = cls._models[pretrained_model]['url']
            utils.download_model(url)


if __name__ == '__main__':
    main()
