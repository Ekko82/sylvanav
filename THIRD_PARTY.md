# Assets and attribution

The MIT license applies to this project's original code and numeric data. It
does not change third-party licenses, grant trademark rights or imply endorsement.

## Drone model

“DJI Inspire 3” by **polyman Studio**, licensed under
[Creative Commons Attribution 4.0 International](https://creativecommons.org/licenses/by/4.0/).

- Source: https://sketchfab.com/3d-models/dji-inspire-3-158e4fa0710f4511be14b73374b5df9b
- Creator: https://sketchfab.com/Polyman_3D
- Changes: normalized size and alignment; optical material adjustments; gimbal
  pitch calibration; rotor pivot rig; recorded pose animation; lighting and rendering.
- This is a third-party depiction, not an official DJI asset or DJI software.
- The source GLB is not stored in Git. Its source URL and expected SHA-256 are
  recorded in [assets/model-source.json](assets/model-source.json).

## Textures and lighting

The selected Poly Haven asset files are
[CC0 1.0 Universal](https://creativecommons.org/publicdomain/zero/1.0/), under
[Poly Haven's asset license](https://polyhaven.com/license).

| Asset | Use | Source |
|---|---|---|
| forest_floor | Ground color, roughness and normal | https://polyhaven.com/a/forest_floor |
| leafy_grass | Ground color, roughness and normal | https://polyhaven.com/a/leafy_grass |
| bark_brown_01 | Bark color, roughness and normal | https://polyhaven.com/a/bark_brown_01 |
| rainforest_trail | HDRI environment lighting/background | https://polyhaven.com/a/rainforest_trail |

[assets/textures.json](assets/textures.json) preserves the authors, original
download URLs, dimensions and SHA-256 hashes. Website thumbnails, example
renders, logos and promotional previews are not redistributed.

## Scene and rendered media

The released editable scene, video, cover and animation preview are distributed
under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/), subject to the
above third-party credits. Project scene composition, procedural forest geometry,
camera choreography and rendering are by **Ekko82**. The depicted drone model
remains the work of **polyman Studio**; the Poly Haven files remain CC0.

Suggested credit when redistributing the scene or rendered media:

> SylvaNav — scene, animation and rendering by Ekko82;
> “DJI Inspire 3” model by polyman Studio (CC BY 4.0), modified for this scene;
> textures and HDRI from Poly Haven (CC0).

Keep the model source and license links above with that credit, and identify
any additional changes you make. Credits are supplied here alongside the media.

## Tools and simulation provenance

Blender and FFmpeg are external tools and are not bundled. Their own licenses
continue to apply. The included 600-frame pose sequence is resampled from a
previous PX4/Gazebo X500 simulation recording. PX4/Gazebo source code and the
original raw recording are not redistributed in this repository.
