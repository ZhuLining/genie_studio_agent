from __future__ import annotations

from gsa_taskflow_executor.runtime.payload_sanitizer import (
    PayloadSanitizerConfig,
    payload_preview,
    sanitize_event_payload,
    sanitize_status_payload,
    summarize_variables,
)


def test_sanitize_payload_redacts_sensitive_and_omits_heavy_fields() -> None:
    payload = {
        "api_token": "secret-token",
        "nested": {"password": "secret-password"},
        "imageBase64": "a" * 1000,
        "raw": {"gdk": ["x"] * 100},
        "message": "hello",
    }

    sanitized = sanitize_event_payload(
        payload,
        config=PayloadSanitizerConfig(max_string_length=16, max_collection_items=8),
    )

    assert sanitized["api_token"] == "[REDACTED]"
    assert sanitized["nested"]["password"] == "[REDACTED]"
    assert sanitized["imageBase64"]["omitted"] is True
    assert sanitized["imageBase64"]["bytes"] == 1000
    assert sanitized["raw"]["omitted"] is True
    assert sanitized["raw"]["reason"] == "heavy_field"
    assert sanitized["message"] == "hello"


def test_sanitize_payload_truncates_strings_and_collections() -> None:
    sanitized = sanitize_event_payload(
        {
            "long": "x" * 32,
            "items": [1, 2, 3, 4],
        },
        config=PayloadSanitizerConfig(max_string_length=8, max_collection_items=2),
    )

    assert sanitized["long"] == "xxxxxxxx...[truncated]"
    assert sanitized["items"] == {
        "items": [1, 2],
        "omitted_items": 2,
        "total_items": 4,
    }


def test_status_payload_summarizes_variables_by_default() -> None:
    payload = {
        "task_state": "OVER",
        "variables": {
            "节点1": {
                "detail": {
                    "status": "success",
                    "node_type": "worker",
                    "outputs": {"value": 1},
                }
            }
        },
    }

    sanitized = sanitize_status_payload(payload)

    assert sanitized["variables"] == {
        "summary_only": True,
        "node_count": 1,
        "node_ids": ["节点1"],
        "nodes": {
            "节点1": {
                "status": "success",
                "node_type": "worker",
                "output_keys": ["value"],
            }
        },
    }


def test_status_payload_can_keep_small_variables_when_explicitly_enabled() -> None:
    payload = {
        "variables": {
            "节点1": {
                "detail": {"outputs": {"value": 1}},
            }
        }
    }

    sanitized = sanitize_status_payload(
        payload,
        config=PayloadSanitizerConfig(include_full_variables=True),
    )

    assert sanitized["variables"]["节点1"]["detail"]["outputs"]["value"] == 1


def test_summarize_variables_handles_non_mapping_records() -> None:
    summary = summarize_variables({"节点1": {"detail": {"outputs": {"x": 1}}}, "节点2": []})

    assert summary["node_count"] == 2
    assert summary["nodes"]["节点1"]["output_keys"] == ["x"]
    assert summary["nodes"]["节点2"] == {"type": "list"}


def test_payload_preview_keeps_length_metadata() -> None:
    preview = payload_preview(
        "0123456789",
        config=PayloadSanitizerConfig(max_string_length=4),
    )

    assert preview == {
        "preview": "0123...[truncated]",
        "characters": 10,
        "bytes": 10,
        "truncated": True,
    }
