//
//  Copyright 2024 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#include "creep_guidance_panel.hpp"

#include <QHBoxLayout>
#include <QHeaderView>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <string>

namespace rviz_plugins
{
using std::placeholders::_1;

CreepGuidancePanel::CreepGuidancePanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * v_layout = new QVBoxLayout;

  // Status label
  status_label_ = new QLabel("Waiting for creep guidance status...");
  status_label_->setAlignment(Qt::AlignCenter);
  v_layout->addWidget(status_label_);

  // Status table
  auto vertical_header = new QHeaderView(Qt::Vertical);
  vertical_header->hide();
  auto horizontal_header = new QHeaderView(Qt::Horizontal);
  horizontal_header->setSectionResizeMode(QHeaderView::Stretch);

  status_table_ = new QTableWidget();
  status_table_->setColumnCount(3);
  status_table_->setHorizontalHeaderLabels({"ID", "Module", "State"});
  status_table_->setVerticalHeader(vertical_header);
  status_table_->setHorizontalHeader(horizontal_header);

  v_layout->addWidget(status_table_);

  setLayout(v_layout);
}

void CreepGuidancePanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // TODO: Update topic name as needed
  sub_creep_status_ = raw_node_->create_subscription<CreepStatusArray>(
    "/creep_guidance/status", 1, std::bind(&CreepGuidancePanel::onCreepStatus, this, _1));
}

void CreepGuidancePanel::onCreepStatus(const CreepStatusArray::ConstSharedPtr msg)
{
  status_table_->clearContents();
  status_label_->setText(
    QString::fromStdString("Creep Statuses: " + std::to_string(msg->statuses.size())));

  if (msg->statuses.empty()) {
    status_table_->update();
    return;
  }

  status_table_->setRowCount(msg->statuses.size());

  int cnt = 0;
  for ([[maybe_unused]] const auto & status : msg->statuses) {
    // ID
    {
      auto label = new QLabel(QString::number(cnt));
      label->setAlignment(Qt::AlignCenter);
      status_table_->setCellWidget(cnt, 0, label);
    }

    // Module
    {
      auto label = new QLabel(QString::fromStdString("Module"));
      label->setAlignment(Qt::AlignCenter);
      status_table_->setCellWidget(cnt, 1, label);
    }

    // State
    {
      auto label = new QLabel(QString::fromStdString("State"));
      label->setAlignment(Qt::AlignCenter);
      status_table_->setCellWidget(cnt, 2, label);
    }

    cnt++;
  }

  status_table_->update();
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::CreepGuidancePanel, rviz_common::Panel)
