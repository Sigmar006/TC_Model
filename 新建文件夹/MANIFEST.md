# Code Line Statistics Split

Scope: copied `.h` and `.cpp` files only. Original project files were not moved.

- `infrared_sensor`: SBIRS infrared sensor API and implementation files.
- `lrdr_radar`: LRDR acceptance radar code as represented by the RIR module in this repo.
- `shared`: direct shared source/header dependencies included by both SBIRS and RIR.

| category | files | .h files | .cpp files | lines | nonblank lines |
| --- | ---: | ---: | ---: | ---: | ---: |
| infrared_sensor | 86 | 52 | 34 | 13038 | 11759 |
| lrdr_radar | 94 | 59 | 35 | 16319 | 14655 |
| shared | 22 | 19 | 3 | 3514 | 3136 |

## Comment Rate

Supplemental architecture comments were added to the infrared and LRDR snapshot copies only;
the original files under the repository `include/` and `src/` trees were not changed.
The calculation excludes blank lines, counts standalone comment lines as comments, and counts
mixed code/comment lines as code.

| category | code lines | comment lines | comment rate |
| --- | ---: | ---: | ---: |
| infrared_sensor | 8474 | 3285 | 27.94% |
| lrdr_radar | 10658 | 3997 | 27.27% |

Machine-readable results are in `comment_rate_summary.csv`.
