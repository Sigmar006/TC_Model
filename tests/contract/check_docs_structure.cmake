# check_docs_structure.cmake
#
# Guard repository documentation shape:
#   - docs/ has common, review, practice, and the module directories
#   - each business module uses the design-doc set (design.md + boundaries.md +
#     data-flow.md + algorithms.md), all declaring Status: active
#   - common uses the approved common-document set (contract.md + open_questions.md)
#   - review uses draft Markdown files only, with no nested directory tree
#   - practice holds flat active-status Markdown for engineering/infra topics
#   - explainers holds flat HTML explainer pages (canvas figures); no content rules yet
#   - top-level loose Markdown files and legacy archive/migration folders do not reappear
#
# Per docs/common/contract.md §文档结构, each business module keeps a design-doc
# set: design.md (navigation entry + module positioning), boundaries.md
# (module-level boundaries, non-goals, change rules), data-flow.md (data flow,
# I/O, state ownership) and algorithms.md (algorithm registry + per-algorithm
# implementation boundaries). The prior README/contract/decisions/history set
# has been collapsed into this set, and common keeps contract.md (public
# contract) and open_questions.md (non-normative cross-module open questions).
# docs/review is the only approved draft/review holding area; docs/practice holds
# active engineering-practice docs (build, test, examples, batch validation).

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED SOURCE_DIR)
  message(FATAL_ERROR "SOURCE_DIR is required")
endif()
# 规范化为绝对路径：GLOB ... RELATIVE 在 -P 脚本模式下，当 base 为相对路径时
# 返回空列表（静默放行违规）。对绝对输入幂等，仅兜底手动调用。
get_filename_component(SOURCE_DIR "${SOURCE_DIR}" ABSOLUTE)

set(ALLOWED_DOC_DIRS
    "airborne_radar"
    "common"
    "electro_optical_sensor"
    "electronic_countermeasure"
    "electronic_surveillance_radar"
    "flight_dynamic"
    "fusion"
    "navigation"
    "practice"
    "remote_identification_radar"
    "review"
    "sar"
    "sbirs_sensor"
    "threat_assessment"
    "target_inference"
    "precision_evaluation"
    "explainers")

set(BUSINESS_MODULE_DIRS
    "airborne_radar"
    "electro_optical_sensor"
    "electronic_countermeasure"
    "electronic_surveillance_radar"
    "flight_dynamic"
    "fusion"
    "navigation"
    "remote_identification_radar"
    "precision_evaluation"
    "sar"
    "sbirs_sensor"
    "threat_assessment"
    "target_inference")

set(MODULE_DOC_FILES
    "design.md"
    "boundaries.md"
    "data-flow.md"
    "algorithms.md")

set(COMMON_DOC_FILES
    "contract.md"
    "session_contract.md"
    "open_questions.md"
    "rf_architecture.md"
    "issue_codes.md")

set(VIOLATIONS "")

file(GLOB _docs_children RELATIVE "${SOURCE_DIR}/docs" "${SOURCE_DIR}/docs/*")
foreach(child ${_docs_children})
  if(IS_DIRECTORY "${SOURCE_DIR}/docs/${child}")
    list(FIND ALLOWED_DOC_DIRS "${child}" _allowed_dir_idx)
    if(_allowed_dir_idx EQUAL -1)
      list(APPEND VIOLATIONS "docs/${child}: unexpected docs directory")
    endif()
  elseif(child MATCHES "\\.md$" AND NOT child STREQUAL "README.md")
    # docs/README.md is the navigation entry pinned by CLAUDE.md; all other
    # top-level Markdown stays disallowed.
    list(APPEND VIOLATIONS "docs/${child}: top-level Markdown files are not allowed; place content under docs/common or a module directory")
  endif()
endforeach()

foreach(module ${BUSINESS_MODULE_DIRS})
  set(module_dir "${SOURCE_DIR}/docs/${module}")
  if(NOT IS_DIRECTORY "${module_dir}")
    list(APPEND VIOLATIONS "docs/${module}: missing module docs directory")
    continue()
  endif()

  # design.md is mandatory and must be active.
  set(design_file "${module_dir}/design.md")
  if(NOT EXISTS "${design_file}")
    list(APPEND VIOLATIONS "docs/${module}/design.md: missing module navigation doc")
  else()
    file(STRINGS "${design_file}" _head LIMIT_COUNT 6)
    list(FIND _head "Status: active" _status_idx)
    if(_status_idx EQUAL -1)
      list(APPEND VIOLATIONS "docs/${module}/design.md: must declare 'Status: active' near the top")
    endif()
  endif()

  # Optional design-doc siblings (boundaries.md, data-flow.md, algorithms.md):
  # if present they must declare Status: active; absence is allowed (e.g. flight_dynamic
  # has no data-flow.md, and modules migrate incrementally).
  foreach(filename boundaries.md data-flow.md algorithms.md)
    set(doc_file "${module_dir}/${filename}")
    if(EXISTS "${doc_file}")
      file(STRINGS "${doc_file}" _head LIMIT_COUNT 6)
      list(FIND _head "Status: active" _status_idx)
      if(_status_idx EQUAL -1)
        list(APPEND VIOLATIONS "docs/${module}/${filename}: must declare 'Status: active' near the top")
      endif()
    endif()
  endforeach()

  file(GLOB_RECURSE _module_docs RELATIVE "${module_dir}" "${module_dir}/*.md")
  foreach(rel_doc ${_module_docs})
    list(FIND MODULE_DOC_FILES "${rel_doc}" _allowed_doc_idx)
    if(_allowed_doc_idx EQUAL -1)
      list(APPEND VIOLATIONS "docs/${module}/${rel_doc}: module docs must use the design-doc set (design.md/boundaries.md/data-flow.md/algorithms.md)")
    endif()
  endforeach()
endforeach()

set(common_dir "${SOURCE_DIR}/docs/common")
if(NOT IS_DIRECTORY "${common_dir}")
  list(APPEND VIOLATIONS "docs/common: missing common docs directory")
else()
  foreach(filename ${COMMON_DOC_FILES})
    set(doc_file "${common_dir}/${filename}")
    if(NOT EXISTS "${doc_file}")
      list(APPEND VIOLATIONS "docs/common/${filename}: missing common doc")
      continue()
    endif()
    file(STRINGS "${doc_file}" _head LIMIT_COUNT 6)
    list(FIND _head "Status: active" _status_idx)
    if(_status_idx EQUAL -1)
      list(APPEND VIOLATIONS "docs/common/${filename}: must declare 'Status: active' near the top")
    endif()
  endforeach()

  file(GLOB_RECURSE _common_docs RELATIVE "${common_dir}" "${common_dir}/*.md")
  foreach(rel_doc ${_common_docs})
    list(FIND COMMON_DOC_FILES "${rel_doc}" _allowed_doc_idx)
    if(_allowed_doc_idx EQUAL -1)
      list(APPEND VIOLATIONS "docs/common/${rel_doc}: unexpected common doc")
    endif()
  endforeach()
endif()

set(review_dir "${SOURCE_DIR}/docs/review")
if(IS_DIRECTORY "${review_dir}")
  file(GLOB_RECURSE _review_docs RELATIVE "${review_dir}" "${review_dir}/*.md")
  foreach(rel_doc ${_review_docs})
    if(rel_doc MATCHES "/")
      list(APPEND VIOLATIONS "docs/review/${rel_doc}: review docs must stay flat")
    endif()
    set(doc_file "${review_dir}/${rel_doc}")
    file(STRINGS "${doc_file}" _head LIMIT_COUNT 6)
    # 收口惯例（docs/review 完成文档收口策略）：评审文档须在头部声明状态——
    # 进行中为 "Status: draft"，已收口完结为 "Status: final"（含 Completed 行）。
    set(_status_declared FALSE)
    foreach(_line IN LISTS _head)
      if(_line MATCHES "^Status: (draft|final)$")
        set(_status_declared TRUE)
      endif()
    endforeach()
    if(NOT _status_declared)
      list(APPEND VIOLATIONS "docs/review/${rel_doc}: review docs must declare 'Status: draft' (open) or 'Status: final' (closed) near the top")
    endif()
  endforeach()

  file(GLOB _review_children RELATIVE "${review_dir}" "${review_dir}/*")
  foreach(child ${_review_children})
    if(IS_DIRECTORY "${review_dir}/${child}")
      list(APPEND VIOLATIONS "docs/review/${child}: review subdirectories are not allowed")
    elseif(NOT child MATCHES "\\.md$")
      list(APPEND VIOLATIONS "docs/review/${child}: review entries must be Markdown files")
    endif()
  endforeach()
endif()

# docs/practice: flat active-status Markdown for engineering/infra topics.
set(practice_dir "${SOURCE_DIR}/docs/practice")
if(IS_DIRECTORY "${practice_dir}")
  file(GLOB_RECURSE _practice_docs RELATIVE "${practice_dir}" "${practice_dir}/*.md")
  foreach(rel_doc ${_practice_docs})
    if(rel_doc MATCHES "/")
      list(APPEND VIOLATIONS "docs/practice/${rel_doc}: practice docs must stay flat")
    endif()
    set(doc_file "${practice_dir}/${rel_doc}")
    file(STRINGS "${doc_file}" _head LIMIT_COUNT 6)
    list(FIND _head "Status: active" _active_idx)
    if(_active_idx EQUAL -1)
      list(APPEND VIOLATIONS "docs/practice/${rel_doc}: practice docs must declare 'Status: active' near the top")
    endif()
  endforeach()

  file(GLOB _practice_children RELATIVE "${practice_dir}" "${practice_dir}/*")
  foreach(child ${_practice_children})
    if(IS_DIRECTORY "${practice_dir}/${child}")
      list(APPEND VIOLATIONS "docs/practice/${child}: practice subdirectories are not allowed")
    elseif(NOT child MATCHES "\\.md$")
      list(APPEND VIOLATIONS "docs/practice/${child}: practice entries must be Markdown files")
    endif()
  endforeach()
endif()

if(VIOLATIONS)
  set(_err "Documentation structure violations:\n\n")
  foreach(v ${VIOLATIONS})
    string(APPEND _err "  ${v}\n")
  endforeach()
  message(FATAL_ERROR "${_err}")
endif()

list(LENGTH COMMON_DOC_FILES _common_count)
list(LENGTH MODULE_DOC_FILES _module_doc_count)
list(LENGTH BUSINESS_MODULE_DIRS _module_count)
if(IS_DIRECTORY "${SOURCE_DIR}/docs/review")
  file(GLOB _review_docs_count "${SOURCE_DIR}/docs/review/*.md")
  list(LENGTH _review_docs_count _review_count)
else()
  set(_review_count 0)
endif()
if(IS_DIRECTORY "${SOURCE_DIR}/docs/practice")
  file(GLOB _practice_docs_count "${SOURCE_DIR}/docs/practice/*.md")
  list(LENGTH _practice_docs_count _practice_count)
else()
  set(_practice_count 0)
endif()
math(EXPR _module_doc_total "${_module_count} * ${_module_doc_count}")
message(STATUS "[docs-structure] common=${_common_count}, modules=${_module_count}, module_docs=${_module_doc_total}, review=${_review_count}, practice=${_practice_count}, violations=0")
