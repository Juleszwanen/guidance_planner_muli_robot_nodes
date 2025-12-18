#!/usr/bin/env python3
"""
Centralized logging system for middleware nodes.
Buffers startup logs and displays them in a clean, organized format.
"""

import rospy
from typing import List, Dict, Any


class MiddlewareLogger:
    """Centralized logger that buffers initialization logs and displays them once."""
    
    def __init__(self, node_name: str, namespace: str):
        self.node_name = node_name
        self.namespace = namespace
        self._init_buffer = []
        self._config = {}
        self._connections = []
        self._subscriptions = []
        self._publications = []
        self._errors = []
        self._warnings = []
        
    def add_config(self, key: str, value: Any):
        """Add configuration parameter."""
        self._config[key] = value
    
    def add_connection(self, endpoint: str, description: str = ""):
        """Add ZeroMQ connection."""
        self._connections.append((endpoint, description))
    
    def add_subscription(self, topic: str, msg_type: str = ""):
        """Add ROS/ZeroMQ subscription."""
        self._subscriptions.append((topic, msg_type))
    
    def add_publication(self, topic: str, msg_type: str = ""):
        """Add ROS/ZeroMQ publication."""
        self._publications.append((topic, msg_type))
    
    def add_error(self, message: str):
        """Add error message."""
        self._errors.append(message)
    
    def add_warning(self, message: str):
        """Add warning message."""
        self._warnings.append(message)
    
    def log_startup_summary(self):
        """Display all buffered initialization information in a clean summary."""
        # Build summary sections
        lines = []
        lines.append("=" * 80)
        lines.append(f"  {self.node_name} - Initialization Summary")
        lines.append(f"  Robot: {self.namespace}")
        lines.append("=" * 80)
        
        # Configuration section
        if self._config:
            lines.append("\n[Configuration]")
            for key, value in self._config.items():
                lines.append(f"  • {key}: {value}")
        
        # Connections section
        if self._connections:
            lines.append("\n[ZeroMQ Connections]")
            for endpoint, desc in self._connections:
                if desc:
                    lines.append(f"  • {endpoint} ({desc})")
                else:
                    lines.append(f"  • {endpoint}")
        
        # Subscriptions section
        if self._subscriptions:
            lines.append(f"\n[Subscriptions] ({len(self._subscriptions)} topics)")
            for topic, msg_type in self._subscriptions:
                if msg_type:
                    lines.append(f"  • {topic} [{msg_type}]")
                else:
                    lines.append(f"  • {topic}")
        
        # Publications section
        if self._publications:
            lines.append(f"\n[Publications] ({len(self._publications)} topics)")
            for topic, msg_type in self._publications:
                if msg_type:
                    lines.append(f"  • {topic} [{msg_type}]")
                else:
                    lines.append(f"  • {topic}")
        
        # Warnings section
        if self._warnings:
            lines.append("\n[Warnings]")
            for warning in self._warnings:
                lines.append(f"  ⚠ {warning}")
        
        # Errors section
        if self._errors:
            lines.append("\n[Errors]")
            for error in self._errors:
                lines.append(f"  ✗ {error}")
        
        # Status footer
        lines.append("\n" + "-" * 80)
        if self._errors:
            lines.append(f"  Status: FAILED ({len(self._errors)} error(s))")
        elif self._warnings:
            lines.append(f"  Status: READY (with {len(self._warnings)} warning(s))")
        else:
            lines.append("  Status: READY ✓")
        lines.append("=" * 80)
        
        # Log everything as a single info message
        rospy.loginfo("\n" + "\n".join(lines))
    
    def log_runtime_event(self, event_type: str, message: str, level: str = "debug"):
        """Log runtime events (non-initialization) with consistent formatting."""
        formatted_msg = f"{self.namespace}: [{event_type}] {message}"
        
        if level == "error":
            rospy.logerr(formatted_msg)
        elif level == "warn":
            rospy.logwarn(formatted_msg)
        elif level == "info":
            rospy.loginfo(formatted_msg)
        else:  # debug
            rospy.logdebug(formatted_msg)
